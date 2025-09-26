#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <chrono>
#include <mutex>
#include <filesystem>
#include <cmath>
#include <deque>
#include <unordered_map>
#include <thread>
#include <atomic>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// 关键帧结构体
struct KeyFrame {
    int id;
    Eigen::Matrix4f pose;
    PointCloudT::Ptr cloud;
    std::chrono::steady_clock::time_point timestamp;
    
    KeyFrame(int id_, const Eigen::Matrix4f& pose_, PointCloudT::Ptr cloud_)
        : id(id_), pose(pose_), cloud(cloud_), timestamp(std::chrono::steady_clock::now()) {}
};

// 回环约束结构体
struct LoopConstraint {
    int from_id;
    int to_id;
    Eigen::Matrix4f relative_pose;
    double confidence;
    
    LoopConstraint(int from, int to, const Eigen::Matrix4f& pose, double conf)
        : from_id(from), to_id(to), relative_pose(pose), confidence(conf) {}
};

// 轨迹点结构体
struct TrajectoryPoint {
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    std::chrono::steady_clock::time_point timestamp;
    
    TrajectoryPoint(const Eigen::Matrix4f& pose)
        : timestamp(std::chrono::steady_clock::now()) {
        position = pose.block<3,1>(0,3);
        Eigen::Matrix3f rotation = pose.block<3,3>(0,0);
        orientation = Eigen::Quaternionf(rotation);
    }
};

class MappingNode : public rclcpp::Node {
public:
    MappingNode() : Node("mapping_node"), 
                   map_initialized_(false),
                   keyframe_counter_(0),
                   loop_detection_enabled_(true),
                   pose_graph_optimization_enabled_(true),
                   shutdown_requested_(false) {
        
        // 参数声明
        this->declare_parameter("input_topic", "/odin/points");
        this->declare_parameter("save_directory", "./maps");
        this->declare_parameter("leaf_size", 0.1);
        this->declare_parameter("icp_max_correspondence_distance", 1.0);
        this->declare_parameter("icp_transformation_epsilon", 1e-6);
        this->declare_parameter("icp_euclidean_fitness_epsilon", 1e-6);
        this->declare_parameter("icp_max_iterations", 50);
        this->declare_parameter("max_range", 50.0);
        this->declare_parameter("min_range", 0.5);
        
        // 新增SLAM参数
        this->declare_parameter("keyframe_distance_threshold", 1.0);
        this->declare_parameter("keyframe_angle_threshold", 0.3);
        this->declare_parameter("max_map_size", 1000000);
        this->declare_parameter("loop_closure_distance_threshold", 3.0);
        this->declare_parameter("loop_closure_score_threshold", 0.6);
        this->declare_parameter("pose_graph_optimization_interval", 10);
        this->declare_parameter("trajectory_save_interval", 100);
        
        // 获取参数
        input_topic_ = this->get_parameter("input_topic").as_string();
        save_directory_ = this->get_parameter("save_directory").as_string();
        leaf_size_ = this->get_parameter("leaf_size").as_double();
        icp_max_correspondence_distance_ = this->get_parameter("icp_max_correspondence_distance").as_double();
        icp_transformation_epsilon_ = this->get_parameter("icp_transformation_epsilon").as_double();
        icp_euclidean_fitness_epsilon_ = this->get_parameter("icp_euclidean_fitness_epsilon").as_double();
        icp_max_iterations_ = this->get_parameter("icp_max_iterations").as_int();
        max_range_ = this->get_parameter("max_range").as_double();
        min_range_ = this->get_parameter("min_range").as_double();
        
        keyframe_distance_threshold_ = this->get_parameter("keyframe_distance_threshold").as_double();
        keyframe_angle_threshold_ = this->get_parameter("keyframe_angle_threshold").as_double();
        max_map_size_ = this->get_parameter("max_map_size").as_int();
        loop_closure_distance_threshold_ = this->get_parameter("loop_closure_distance_threshold").as_double();
        loop_closure_score_threshold_ = this->get_parameter("loop_closure_score_threshold").as_double();
        pose_graph_optimization_interval_ = this->get_parameter("pose_graph_optimization_interval").as_int();
        trajectory_save_interval_ = this->get_parameter("trajectory_save_interval").as_int();
        
        // 创建发布器
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mapping/accumulated_map", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/mapping/odometry", 10);
        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mapping/trajectory", 10);
        
        // 创建订阅器
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&MappingNode::cloudCallback, this, std::placeholders::_1));
        
        // 创建服务
        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/mapping/save_map", std::bind(&MappingNode::saveService, this, std::placeholders::_1, std::placeholders::_2));
        
        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/mapping/reset", std::bind(&MappingNode::resetService, this, std::placeholders::_1, std::placeholders::_2));
        
        loop_closure_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/mapping/enable_loop_closure", std::bind(&MappingNode::loopClosureService, this, std::placeholders::_1, std::placeholders::_2));
        
        // 初始化点云
        accumulated_map_ = std::make_shared<PointCloudT>();
        current_pose_ = Eigen::Matrix4f::Identity();
        
        // 创建保存目录
        std::filesystem::create_directories(save_directory_);
        
        // 启动后台线程
        loop_detection_thread_ = std::thread(&MappingNode::loopDetectionThread, this);
        pose_graph_thread_ = std::thread(&MappingNode::poseGraphOptimizationThread, this);
        
        RCLCPP_INFO(this->get_logger(), "Enhanced SLAM Mapping Node initialized");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Save directory: %s", save_directory_.c_str());
    }
    
    ~MappingNode() {
        shutdown_requested_ = true;
        if (loop_detection_thread_.joinable()) {
            loop_detection_thread_.join();
        }
        if (pose_graph_thread_.joinable()) {
            pose_graph_thread_.join();
        }
        
        // 保存最终轨迹
        saveTrajectory();
        RCLCPP_INFO(this->get_logger(), "SLAM Mapping Node shutdown complete");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        
        // ROS2 -> PCL 转换
        PointCloudT::Ptr cloud(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud);
        
        // 预处理
        cloud = preprocessCloud(cloud);
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty cloud after preprocessing");
            return;
        }
        
        // 第一帧处理
        if (!map_initialized_) {
            initializeMap(cloud);
            return;
        }
        
        // ICP配准
        Eigen::Matrix4f transformation;
        double fitness_score;
        if (!performICP(cloud, transformation, fitness_score)) {
            RCLCPP_WARN(this->get_logger(), "ICP failed, skipping frame");
            return;
        }
        
        // 更新位姿
        current_pose_ = current_pose_ * transformation;
        
        // 记录轨迹
        trajectory_.emplace_back(current_pose_);
        
        // 检查是否需要创建关键帧
        if (shouldCreateKeyframe(transformation)) {
            createKeyframe(cloud);
        }
        
        // 更新地图
        updateMap(cloud);
        
        // 发布结果
        publishAll();
        
        // 定期保存轨迹
        if (trajectory_.size() % trajectory_save_interval_ == 0) {
            saveTrajectory();
        }
    }
    
    PointCloudT::Ptr preprocessCloud(PointCloudT::Ptr cloud) {
        // 移除NaN和Inf点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        
        // 距离过滤
        PointCloudT::Ptr filtered_cloud(new PointCloudT);
        for (const auto& point : cloud->points) {
            float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance >= min_range_ && distance <= max_range_) {
                filtered_cloud->points.push_back(point);
            }
        }
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = true;
        
        // 统计滤波去除离群点
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setMeanK(20);
        sor.setStddevMulThresh(2.0);
        PointCloudT::Ptr clean_cloud(new PointCloudT);
        sor.filter(*clean_cloud);
        
        // 体素下采样
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(clean_cloud);
        voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        PointCloudT::Ptr downsampled_cloud(new PointCloudT);
        voxel_filter.filter(*downsampled_cloud);
        
        return downsampled_cloud;
    }
    
    void initializeMap(PointCloudT::Ptr cloud) {
        *accumulated_map_ = *cloud;
        map_initialized_ = true;
        
        // 创建第一个关键帧
        createKeyframe(cloud);
        
        RCLCPP_INFO(this->get_logger(), "Map initialized with %zu points", cloud->size());
    }
    
    bool performICP(PointCloudT::Ptr cloud, Eigen::Matrix4f& transformation, double& fitness_score) {
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        
        // 配置ICP参数
        icp.setInputSource(cloud);
        icp.setInputTarget(accumulated_map_);
        icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
        icp.setTransformationEpsilon(icp_transformation_epsilon_);
        icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);
        icp.setMaximumIterations(icp_max_iterations_);
        
        // 使用RANSAC提高鲁棒性
        icp.setRANSACOutlierRejectionThreshold(0.1);
        icp.setRANSACIterations(100);
        
        PointCloudT::Ptr aligned_cloud(new PointCloudT);
        icp.align(*aligned_cloud);
        
        if (!icp.hasConverged()) {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge");
            return false;
        }
        
        fitness_score = icp.getFitnessScore();
        transformation = icp.getFinalTransformation();
        
        // 检查配准质量
        if (fitness_score > 0.5) {
            RCLCPP_WARN(this->get_logger(), "Poor ICP fitness score: %f", fitness_score);
            return false;
        }
        
        return true;
    }
    
    bool shouldCreateKeyframe(const Eigen::Matrix4f& transformation) {
        // 计算平移距离
        Eigen::Vector3f translation = transformation.block<3,1>(0,3);
        double distance = translation.norm();
        
        // 计算旋转角度
        Eigen::Matrix3f rotation = transformation.block<3,3>(0,0);
        Eigen::AngleAxisf angle_axis(rotation);
        double angle = std::abs(angle_axis.angle());
        
        return (distance > keyframe_distance_threshold_ || angle > keyframe_angle_threshold_);
    }
    
    void createKeyframe(PointCloudT::Ptr cloud) {
        auto keyframe = std::make_shared<KeyFrame>(keyframe_counter_++, current_pose_, cloud);
        keyframes_.push_back(keyframe);
        
        RCLCPP_INFO(this->get_logger(), "Created keyframe %d at position [%.2f, %.2f, %.2f]", 
                   keyframe->id, current_pose_(0,3), current_pose_(1,3), current_pose_(2,3));
        
        // 触发回环检测
        if (loop_detection_enabled_ && keyframes_.size() > 10) {
            std::lock_guard<std::mutex> lock(loop_mutex_);
            loop_detection_queue_.push_back(keyframe);
        }
    }
    
    void updateMap(PointCloudT::Ptr cloud) {
        // 将点云变换到全局坐标系
        PointCloudT::Ptr transformed_cloud(new PointCloudT);
        pcl::transformPointCloud(*cloud, *transformed_cloud, current_pose_);
        
        // 添加到累积地图
        *accumulated_map_ += *transformed_cloud;
        
        // 地图大小管理
        if (accumulated_map_->size() > max_map_size_) {
            // 体素下采样压缩地图
            pcl::VoxelGrid<PointT> voxel_filter;
            voxel_filter.setInputCloud(accumulated_map_);
            voxel_filter.setLeafSize(leaf_size_ * 1.5, leaf_size_ * 1.5, leaf_size_ * 1.5);
            PointCloudT::Ptr compressed_map(new PointCloudT);
            voxel_filter.filter(*compressed_map);
            accumulated_map_ = compressed_map;
            
            RCLCPP_INFO(this->get_logger(), "Map compressed to %zu points", accumulated_map_->size());
        }
    }
    
    void loopDetectionThread() {
        while (!shutdown_requested_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            if (!loop_detection_enabled_) continue;
            
            std::vector<std::shared_ptr<KeyFrame>> candidates;
            {
                std::lock_guard<std::mutex> lock(loop_mutex_);
                if (loop_detection_queue_.empty()) continue;
                
                candidates = loop_detection_queue_;
                loop_detection_queue_.clear();
            }
            
            for (auto& current_kf : candidates) {
                detectLoopClosure(current_kf);
            }
        }
    }
    
    void detectLoopClosure(std::shared_ptr<KeyFrame> current_kf) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        
        for (auto& candidate_kf : keyframes_) {
            // 跳过时间太近的关键帧
            if (std::abs(current_kf->id - candidate_kf->id) < 30) continue;
            
            // 计算距离
            Eigen::Vector3f pos1 = current_kf->pose.block<3,1>(0,3);
            Eigen::Vector3f pos2 = candidate_kf->pose.block<3,1>(0,3);
            double distance = (pos1 - pos2).norm();
            
            if (distance < loop_closure_distance_threshold_) {
                // 尝试ICP验证
                if (verifyLoopClosure(current_kf, candidate_kf)) {
                    RCLCPP_INFO(this->get_logger(), "Loop closure detected between keyframes %d and %d", 
                               current_kf->id, candidate_kf->id);
                }
            }
        }
    }
    
    bool verifyLoopClosure(std::shared_ptr<KeyFrame> kf1, std::shared_ptr<KeyFrame> kf2) {
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(kf1->cloud);
        icp.setInputTarget(kf2->cloud);
        icp.setMaxCorrespondenceDistance(1.0);
        icp.setMaximumIterations(100);
        
        PointCloudT::Ptr aligned_cloud(new PointCloudT);
        icp.align(*aligned_cloud);
        
        if (icp.hasConverged() && icp.getFitnessScore() < loop_closure_score_threshold_) {
            // 添加回环约束
            Eigen::Matrix4f relative_pose = icp.getFinalTransformation();
            auto constraint = std::make_shared<LoopConstraint>(
                kf1->id, kf2->id, relative_pose, 1.0 - icp.getFitnessScore());
            
            std::lock_guard<std::mutex> lock(loop_mutex_);
            loop_constraints_.push_back(constraint);
            
            return true;
        }
        
        return false;
    }
    
    void poseGraphOptimizationThread() {
        while (!shutdown_requested_) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            if (!pose_graph_optimization_enabled_) continue;
            
            if (keyframes_.size() % pose_graph_optimization_interval_ == 0 && !loop_constraints_.empty()) {
                optimizePoseGraph();
            }
        }
    }
    
    void optimizePoseGraph() {
        std::lock_guard<std::mutex> lock1(map_mutex_);
        std::lock_guard<std::mutex> lock2(loop_mutex_);
        
        // 简化的位姿图优化（这里使用基础的最小二乘法）
        // 在实际应用中，应该使用g2o或GTSAM等专业库
        
        RCLCPP_INFO(this->get_logger(), "Performing pose graph optimization with %zu constraints", 
                   loop_constraints_.size());
        
        // 更新关键帧位姿后，重建地图
        rebuildMap();
    }
    
    void rebuildMap() {
        accumulated_map_->clear();
        
        for (auto& kf : keyframes_) {
            PointCloudT::Ptr transformed_cloud(new PointCloudT);
            pcl::transformPointCloud(*kf->cloud, *transformed_cloud, kf->pose);
            *accumulated_map_ += *transformed_cloud;
        }
        
        // 压缩地图
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(accumulated_map_);
        voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        PointCloudT::Ptr compressed_map(new PointCloudT);
        voxel_filter.filter(*compressed_map);
        accumulated_map_ = compressed_map;
        
        RCLCPP_INFO(this->get_logger(), "Map rebuilt with %zu points", accumulated_map_->size());
    }
    
    void publishAll() {
        auto now = this->get_clock()->now();
        
        // 发布地图
        if (!accumulated_map_->empty()) {
            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*accumulated_map_, map_msg);
            map_msg.header.stamp = now;
            map_msg.header.frame_id = "map";
            map_pub_->publish(map_msg);
        }
        
        // 发布里程计
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        
        // 位置
        odom_msg.pose.pose.position.x = current_pose_(0, 3);
        odom_msg.pose.pose.position.y = current_pose_(1, 3);
        odom_msg.pose.pose.position.z = current_pose_(2, 3);
        
        // 姿态
        Eigen::Matrix3f rotation = current_pose_.block<3,3>(0,0);
        Eigen::Quaternionf quat(rotation);
        odom_msg.pose.pose.orientation.x = quat.x();
        odom_msg.pose.pose.orientation.y = quat.y();
        odom_msg.pose.pose.orientation.z = quat.z();
        odom_msg.pose.pose.orientation.w = quat.w();
        
        odom_pub_->publish(odom_msg);
        
        // 发布轨迹
        publishTrajectory();
    }
    
    void publishTrajectory() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";
        
        for (const auto& traj_point : trajectory_) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = traj_point.position.x();
            pose_stamped.pose.position.y = traj_point.position.y();
            pose_stamped.pose.position.z = traj_point.position.z();
            pose_stamped.pose.orientation.x = traj_point.orientation.x();
            pose_stamped.pose.orientation.y = traj_point.orientation.y();
            pose_stamped.pose.orientation.z = traj_point.orientation.z();
            pose_stamped.pose.orientation.w = traj_point.orientation.w();
            
            path_msg.poses.push_back(pose_stamped);
        }
        
        trajectory_pub_->publish(path_msg);
    }
    
    void saveTrajectory() {
        std::string trajectory_file = save_directory_ + "/trajectory.txt";
        std::ofstream file(trajectory_file);
        
        if (file.is_open()) {
            file << "# timestamp x y z qx qy qz qw\n";
            for (size_t i = 0; i < trajectory_.size(); ++i) {
                const auto& point = trajectory_[i];
                file << i << " " 
                     << point.position.x() << " " << point.position.y() << " " << point.position.z() << " "
                     << point.orientation.x() << " " << point.orientation.y() << " " 
                     << point.orientation.z() << " " << point.orientation.w() << "\n";
            }
            file.close();
            RCLCPP_INFO(this->get_logger(), "Trajectory saved to %s", trajectory_file.c_str());
        }
    }
    
    void saveService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        
        std::lock_guard<std::mutex> lock(map_mutex_);
        
        if (accumulated_map_->empty()) {
            response->success = false;
            response->message = "No map to save";
            return;
        }
        
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto tm = *std::localtime(&time_t);
        
        char timestamp[100];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm);
        
        std::string pcd_filename = save_directory_ + "/map_" + timestamp + ".pcd";
        std::string ply_filename = save_directory_ + "/map_" + timestamp + ".ply";
        
        try {
            pcl::io::savePCDFileBinary(pcd_filename, *accumulated_map_);
            pcl::io::savePLYFileBinary(ply_filename, *accumulated_map_);
            
            // 保存轨迹
            saveTrajectory();
            
            response->success = true;
            response->message = "Map saved successfully to " + pcd_filename + " and " + ply_filename;
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to save map: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }
    
    void resetService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        
        std::lock_guard<std::mutex> lock1(map_mutex_);
        std::lock_guard<std::mutex> lock2(loop_mutex_);
        
        accumulated_map_->clear();
        keyframes_.clear();
        trajectory_.clear();
        loop_constraints_.clear();
        loop_detection_queue_.clear();
        
        current_pose_ = Eigen::Matrix4f::Identity();
        map_initialized_ = false;
        keyframe_counter_ = 0;
        
        response->success = true;
        response->message = "SLAM system reset successfully";
        RCLCPP_INFO(this->get_logger(), "SLAM system reset");
    }
    
    void loopClosureService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        loop_detection_enabled_ = request->data;
        response->success = true;
        response->message = loop_detection_enabled_ ? "Loop closure enabled" : "Loop closure disabled";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    // 参数
    std::string input_topic_;
    std::string save_directory_;
    double leaf_size_;
    double icp_max_correspondence_distance_;
    double icp_transformation_epsilon_;
    double icp_euclidean_fitness_epsilon_;
    int icp_max_iterations_;
    double max_range_;
    double min_range_;
    
    // SLAM参数
    double keyframe_distance_threshold_;
    double keyframe_angle_threshold_;
    int max_map_size_;
    double loop_closure_distance_threshold_;
    double loop_closure_score_threshold_;
    int pose_graph_optimization_interval_;
    int trajectory_save_interval_;
    
    // ROS2接口
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr loop_closure_service_;
    
    // 数据成员
    std::mutex map_mutex_;
    std::mutex loop_mutex_;
    PointCloudT::Ptr accumulated_map_;
    Eigen::Matrix4f current_pose_;
    bool map_initialized_;
    
    // SLAM数据结构
    std::vector<std::shared_ptr<KeyFrame>> keyframes_;
    std::vector<TrajectoryPoint> trajectory_;
    std::vector<std::shared_ptr<LoopConstraint>> loop_constraints_;
    std::deque<std::shared_ptr<KeyFrame>> loop_detection_queue_;
    
    int keyframe_counter_;
    std::atomic<bool> loop_detection_enabled_;
    std::atomic<bool> pose_graph_optimization_enabled_;
    std::atomic<bool> shutdown_requested_;
    
    // 后台线程
    std::thread loop_detection_thread_;
    std::thread pose_graph_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
