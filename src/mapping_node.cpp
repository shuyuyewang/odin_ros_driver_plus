#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>           // 注意小写
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>               // removeNaNFromPointCloud
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>
#include <mutex>
#include <filesystem>
#include <cmath>

class MappingNode : public rclcpp::Node {
public:
  MappingNode() : Node("mapping_node") {
    input_topic_    = this->declare_parameter<std::string>("input_topic", "/odin1/cloud_raw");
    leaf_size_      = this->declare_parameter<double>("leaf_size", 0.05); // m
    min_range_      = this->declare_parameter<double>("min_range", 0.1);
    max_range_      = this->declare_parameter<double>("max_range", 80.0);
    fitness_thresh_ = this->declare_parameter<double>("fitness_threshold", std::max(0.02, 4.0*leaf_size_*leaf_size_)); // 与分辨率相关
    save_dir_       = this->declare_parameter<std::string>("save_dir", "mapping_maps");
    save_format_    = this->declare_parameter<std::string>("save_format", "ply"); // "pcd" or "ply"
    output_frame_   = this->declare_parameter<std::string>("output_frame", "map");

    map_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mapping/accumulated_map", rclcpp::QoS(1).transient_local());
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/mapping/odom", rclcpp::QoS(10));
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MappingNode::cloudCallback, this, std::placeholders::_1));

    save_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/mapping/save_map", std::bind(&MappingNode::saveService, this, std::placeholders::_1, std::placeholders::_2));

    std::filesystem::create_directories(save_dir_);
    RCLCPP_INFO(this->get_logger(),
      "MappingNode started. Subscribing: %s, publishing: /mapping/accumulated_map (frame: %s)",
      input_topic_.c_str(), output_frame_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // ROS2 → PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    // 清 NaN/Inf
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);
    if (cloud->empty()) return;

    // 距离筛选（用平方距离，省 sqrt）
    const double min_r2 = min_range_ * min_range_;
    const double max_r2 = max_range_ * max_range_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ranged(new pcl::PointCloud<pcl::PointXYZ>());
    ranged->reserve(cloud->size());
    for (const auto &p : *cloud) {
      const double r2 = static_cast<double>(p.x)*p.x + static_cast<double>(p.y)*p.y + static_cast<double>(p.z)*p.z;
      if (std::isfinite(r2) && r2 >= min_r2 && r2 <= max_r2) {
        ranged->push_back(p);
      }
    }
    if (ranged->empty()) return;

    // 体素下采样
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    vg.setInputCloud(ranged);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*filtered);
    if (filtered->empty()) return;

    std::lock_guard<std::mutex> lk(map_mutex_);

    if (!map_initialized_) {
      map_cloud_ = filtered;
      map_initialized_ = true;
      T_global_.setIdentity(); // 首帧当作 map 坐标系
      publishAll(msg->header.stamp);
      return;
    }

    // ICP 配置：对应距离与 leaf_size 同量级的数倍，更稳
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(filtered);
    icp.setInputTarget(map_cloud_);
    icp.setMaximumIterations(50);
    icp.setMaxCorrespondenceDistance(std::max(0.2, 6.0 * leaf_size_)); // 经验值
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-4);
    // 如需更强鲁棒性，可打开 RANSAC 外点抑制：
    // icp.setRANSACOutlierRejectionThreshold(1.5 * leaf_size_);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    // 若有外部里程计/IMU预测，可把预测位姿作为初值 icp.align(aligned, T_pred);
    icp.align(aligned);

    const bool ok = icp.hasConverged();
    const double fitness = icp.getFitnessScore();

    if (!ok || fitness > fitness_thresh_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "ICP failed/poor: converged=%d, fitness=%.5f (thr=%.5f)", (int)ok, fitness, fitness_thresh_);
      // 不融合，仅发布当前地图
      publishAll(msg->header.stamp);
      return;
    }

    // 更新位姿（全局累乘）
    const Eigen::Matrix4f T = icp.getFinalTransformation();
    T_global_ = T_global_ * T;

    // 地图融合：把对齐后的点并入地图
    *map_cloud_ += aligned;

    // 地图压缩：全局再体素化，避免无限膨胀
    vg.setInputCloud(map_cloud_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*down);
    map_cloud_.swap(down);

    publishAll(msg->header.stamp);
  }

  void publishAll(const rclcpp::Time &stamp) {
    // 复制一份点云避免并发读写
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> map_copy;
    {
      if (!map_cloud_ || map_cloud_->empty()) return;
      map_copy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*map_cloud_);
    }

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*map_copy, out);
    out.header.frame_id = output_frame_; // 默认 "map"
    out.header.stamp = stamp;
    map_pub_->publish(out);

    // 发布当前位姿到 Odometry（未发布 TF，如需 TF 可自行添加）
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = output_frame_;
    odom.child_frame_id = "mapping_base";
    odom.header.stamp = stamp;

    Eigen::Matrix3f R = T_global_.block<3,3>(0,0);
    Eigen::Vector3f t = T_global_.block<3,1>(0,3);
    Eigen::Quaternionf q(R);

    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom);
  }

  void saveService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = saveNow();
    res->message = res->success ? "Map saved." : "Save failed.";
  }

  bool saveNow() {
    // 复制一份点云，避免长时间持锁
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> map_copy;
    {
      std::lock_guard<std::mutex> lk(map_mutex_);
      if (!map_cloud_ || map_cloud_->empty()) {
        RCLCPP_WARN(this->get_logger(), "Map is empty, skip save.");
        return false;
      }
      map_copy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*map_cloud_);
    }

    const int64_t sec = this->now().seconds(); // 整秒，避免文件名含小数点
    const std::string path = save_dir_ + "/map_" + std::to_string(sec) + "." + save_format_;

    int ret = 0;
    if (save_format_ == "pcd") {
      ret = pcl::io::savePCDFileBinary(path, *map_copy);
    } else {
      ret = pcl::io::savePLYFileBinary(path, *map_copy);
    }

    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(), "Saved map to %s", path.c_str());
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", path.c_str());
      return false;
    }
  }

  // params
  std::string input_topic_, save_dir_, save_format_, output_frame_;
  double leaf_size_, min_range_, max_range_, fitness_thresh_;

  // ros i/o
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;

  // map & pose
  std::mutex map_mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_{ new pcl::PointCloud<pcl::PointXYZ>() };
  bool map_initialized_{ false };
  Eigen::Matrix4f T_global_{ Eigen::Matrix4f::Identity() };
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  return 0;
}
