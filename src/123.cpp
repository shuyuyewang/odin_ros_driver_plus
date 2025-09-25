#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <cmath>

class SlamMapperNode : public rclcpp::Node {
public:
  SlamMapperNode() : Node("slam_mapper_node"),
                     tf_buffer_(this->get_clock()),
                     tf_listener_(tf_buffer_) {
    cloud_topic_   = this->declare_parameter<std::string>("cloud_topic", "/odin1/cloud_render");
    map_frame_     = this->declare_parameter<std::string>("map_frame",   "slam_map");
    leaf_size_     = this->declare_parameter<double>("leaf_size",        0.05); // m
    min_range_     = this->declare_parameter<double>("min_range",        0.1);
    max_range_     = this->declare_parameter<double>("max_range",        80.0);
    save_dir_      = this->declare_parameter<std::string>("save_dir",    "slam_maps");
    save_format_   = this->declare_parameter<std::string>("save_format", "ply"); // "pcd" or "ply"
    publish_topic_ = this->declare_parameter<std::string>("map_topic",   "/slam/accumulated_map");
    save_interval_ = this->declare_parameter<int>("save_interval_sec",   0); // 0=禁用定时保存

    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic_, rclcpp::QoS(1).transient_local());
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SlamMapperNode::cloudCallback, this, std::placeholders::_1));

    save_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "save_map", std::bind(&SlamMapperNode::saveService, this,
                              std::placeholders::_1, std::placeholders::_2));

    if (save_interval_ > 0) {
      save_timer_ = this->create_wall_timer(
          std::chrono::seconds(save_interval_),
          std::bind(&SlamMapperNode::saveTimer, this));
    }

    std::filesystem::create_directories(save_dir_);
    RCLCPP_INFO(this->get_logger(), "SlamMapperNode started. Subscribing: %s, publishing: %s, map_frame: %s",
                cloud_topic_.c_str(), publish_topic_.c_str(), map_frame_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1) TF 到 map
    sensor_msgs::msg::PointCloud2 cloud_in_map;
    try {
      tf_buffer_.transform(*msg, cloud_in_map, map_frame_, tf2::durationFromSec(0.05));
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF transform failed: %s", e.what());
      return;
    }

    // 2) 转 PCL
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(cloud_in_map, cloud);
    if (cloud.empty()) return;

    // 3) 距离裁剪
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    filtered->reserve(cloud.size());
    for (const auto &p : cloud) {
      const double r = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
      if (std::isfinite(r) && r >= min_range_ && r <= max_range_) filtered->push_back(p);
    }
    if (filtered->empty()) return;

    // 4) 累计融合 + 体素滤波
    std::lock_guard<std::mutex> lk(map_mutex_);
    *map_cloud_ += *filtered;

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    vg.setInputCloud(map_cloud_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*down);
    map_cloud_.swap(down);

    // 5) 发布
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*map_cloud_, out_msg);
    out_msg.header.frame_id = map_frame_;
    out_msg.header.stamp = msg->header.stamp;
    map_pub_->publish(out_msg);
  }

  void saveService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = saveNow();
    res->message = res->success ? "Map saved." : "Save failed.";
  }

  void saveTimer() { (void)saveNow(); }

  bool saveNow() {
    std::string stamp = std::to_string(this->now().seconds());
    std::string path = save_dir_ + "/map_" + stamp + "." + save_format_;
    std::lock_guard<std::mutex> lk(map_mutex_);
    if (map_cloud_->empty()) {
      RCLCPP_WARN(this->get_logger(), "Map is empty, skip save.");
      return false;
    }
    int ret = 0;
    if (save_format_ == "pcd") {
      ret = pcl::io::savePCDFileBinary(path, *map_cloud_);
    } else {
      ret = pcl::io::savePLYFileBinary(path, *map_cloud_);
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
  std::string cloud_topic_, map_frame_, save_dir_, save_format_, publish_topic_;
  double leaf_size_, min_range_, max_range_;
  int save_interval_;

  // ros i/o
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr save_timer_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // map
  std::mutex map_mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ { new pcl::PointCloud<pcl::PointXYZ>() };
};

// ===== main() 保证可执行目标有入口点 =====
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamMapperNode>());
  rclcpp::shutdown();
  return 0;
}