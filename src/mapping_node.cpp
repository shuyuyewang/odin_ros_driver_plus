#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <deque>
#include <optional>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <cmath>

// ------------------------------ MappingNode ------------------------------
class MappingNode : public rclcpp::Node {
public:
  MappingNode() : Node("mapping_node") {
    // === 参数 ===
    input_topic_      = declare_parameter<std::string>("input_topic", "/odin1/cloud_render");
    output_frame_     = declare_parameter<std::string>("output_frame", "map");

    // 预处理/滤波
    leaf_size_        = declare_parameter<double>("leaf_size", 0.05);   // m
    min_range_        = declare_parameter<double>("min_range", 0.30);   // m
    max_range_        = declare_parameter<double>("max_range", 40.0);   // m
    global_voxel_every_n_ = declare_parameter<int>("global_voxel_every_n", 3); // 每N帧做一次全局再体素

    // 里程计/deskew
    enable_deskew_    = declare_parameter<bool>("enable_deskew", false);
    time_field_name_  = declare_parameter<std::string>("deskew_time_field", "time");
    scan_period_      = declare_parameter<double>("scan_period", 0.1);
    deskew_ref_end_   = declare_parameter<bool>("deskew_ref_is_end", true);

    // 单位归一化
    auto_unit_scale_  = declare_parameter<bool>("auto_unit_scale", true);
    unit_scale_param_ = declare_parameter<double>("unit_scale", 1.0); // !=1优先

    // 关键帧策略
    kf_trans_thresh_  = declare_parameter<double>("keyframe_trans_thresh", 0.10); // m
    kf_rot_deg_thresh_= declare_parameter<double>("keyframe_rot_deg_thresh", 6.0); // deg

    // 保存
    save_dir_         = declare_parameter<std::string>("save_dir", "mapping_maps");
    save_format_      = declare_parameter<std::string>("save_format", "ply"); // pcd | ply
    stamp_in_name_    = declare_parameter<bool>("stamp_in_filename", true);
    save_traj_csv_    = declare_parameter<bool>("save_trajectory_csv", true);
    save_period_sec_  = declare_parameter<double>("save_period", 0.0); // >0 开启定时保存

    debug_log_        = declare_parameter<bool>("debug_log", false);

    std::filesystem::create_directories(save_dir_);

    // === ROS I/O ===
    map_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("/mapping/accumulated_map", rclcpp::QoS(1).transient_local());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/mapping/odom", rclcpp::QoS(50));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MappingNode::cloudCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odin1/odometry", rclcpp::QoS(200),
      std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

    save_srv_ = create_service<std_srvs::srv::Trigger>(
      "/mapping/save_map", std::bind(&MappingNode::saveService, this,
                                     std::placeholders::_1, std::placeholders::_2));

    if (save_period_sec_ > 1e-6) {
      auto period = std::chrono::duration<double>(save_period_sec_);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&MappingNode::onTimer, this));
      RCLCPP_INFO(get_logger(), "Auto-save every %.3f s enabled.", save_period_sec_);
    }

    RCLCPP_INFO(get_logger(), "MappingNode (pure-odom) ready. sub=%s, odom=/odin1/odometry, frame=%s",
                input_topic_.c_str(), output_frame_.c_str());
  }

private:
  using PointT = pcl::PointXYZRGB;
  using CloudT = pcl::PointCloud<PointT>;

  struct TimedPose {
    rclcpp::Time stamp;
    Eigen::Matrix4f T; // odom->base
  };
  struct TrajEntry {
    rclcpp::Time stamp;
    Eigen::Vector3f t;
    Eigen::Quaternionf q;
  };

  // --------- 工具 ---------
  static inline Eigen::Matrix4f poseMsgToMat4(const geometry_msgs::msg::Pose &p){
    Eigen::Quaternionf q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    q.normalize();
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T(0,3) = static_cast<float>(p.position.x);
    T(1,3) = static_cast<float>(p.position.y);
    T(2,3) = static_cast<float>(p.position.z);
    return T;
  }
  static inline Eigen::Matrix4f interpolateSE3(const Eigen::Matrix4f& A, const Eigen::Matrix4f& B, double alpha){
    alpha = std::clamp(alpha, 0.0, 1.0);
    Eigen::Quaternionf qA(Eigen::Matrix3f(A.block<3,3>(0,0)));
    Eigen::Quaternionf qB(Eigen::Matrix3f(B.block<3,3>(0,0)));
    Eigen::Quaternionf q = qA.slerp(static_cast<float>(alpha), qB).normalized();
    Eigen::Vector3f t = (1.0f - static_cast<float>(alpha)) * A.block<3,1>(0,3)
                      + static_cast<float>(alpha) * B.block<3,1>(0,3);
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T.block<3,1>(0,3) = t;
    return T;
  }
  static double median(std::vector<double>& v){
    if (v.empty()) return 0.0;
    size_t n=v.size()/2;
    std::nth_element(v.begin(), v.begin()+n, v.end());
    double m=v[n];
    if (v.size()%2==0){
      std::nth_element(v.begin(), v.begin()+n-1, v.end());
      m=0.5*(m+v[n-1]);
    }
    return m;
  }

  // 读取 cloud：如果无 rgb 则先转 XYZ 再补默认灰色
  bool msgToCloudXYZRGB(const sensor_msgs::msg::PointCloud2& msg, CloudT::Ptr& out, bool& had_rgb) {
    had_rgb = false;
    for (const auto& f : msg.fields) if (f.name=="rgb" || f.name=="rgba") { had_rgb=true; break; }

    if (had_rgb) {
      try { out.reset(new CloudT()); pcl::fromROSMsg(msg, *out); return true; }
      catch (...) { return false; }
    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>());
      try { pcl::fromROSMsg(msg, *xyz); } catch(...) { return false; }
      out.reset(new CloudT()); out->reserve(xyz->size());
      const uint8_t r=200,g=200,b=200;
      const uint32_t rgb = (uint32_t(r)<<16) | (uint32_t(g)<<8) | uint32_t(b);
      float rgb_f; std::memcpy(&rgb_f, &rgb, sizeof(float));
      for (auto &p : *xyz) { PointT q; q.x=p.x; q.y=p.y; q.z=p.z; q.rgb=rgb_f; out->push_back(q); }
      return true;
    }
  }

  // 自动单位：中位半径>100 ≈ 毫米制 → ×0.001
  double decideUnitScale(const CloudT& cloud){
    if (unit_scale_param_>0 && std::abs(unit_scale_param_-1.0)>1e-9) return unit_scale_param_;
    if (!auto_unit_scale_) return 1.0;
    std::vector<double> r; r.reserve(std::min<size_t>(cloud.size(), 1000));
    size_t step = std::max<size_t>(1, cloud.size()/1000);
    for (size_t i=0;i<cloud.size(); i+=step){
      const auto &p = cloud[i];
      r.push_back(std::sqrt(double(p.x)*p.x + double(p.y)*p.y + double(p.z)*p.z));
    }
    double med = median(r);
    return (med > 100.0) ? 0.001 : 1.0;
  }

  // 提取每点相对时间
  std::vector<double> extractPointTimes(const sensor_msgs::msg::PointCloud2& msg){
    const size_t N = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    std::vector<double> rel(N, 0.0);
    bool has_time=false;
    for (const auto& f: msg.fields) if (f.name==time_field_name_) { has_time=true; break; }

    if (has_time){
      sensor_msgs::PointCloud2ConstIterator<float> it_t(msg, time_field_name_);
      for (size_t i=0; i<N && it_t!=it_t.end(); ++i, ++it_t) rel[i]=static_cast<double>(*it_t);
      double tmin=rel.front(), tmax=rel.front();
      for (double v:rel){ tmin=std::min(tmin,v); tmax=std::max(tmax,v); }
      double tref = deskew_ref_end_ ? tmax : tmin;
      for (double &v:rel) v -= tref;
    }else{
      if (N<=1 || scan_period_<=0.0) return rel;
      for (size_t i=0;i<N;++i){
        double frac = static_cast<double>(i) / static_cast<double>(N-1);
        rel[i] = frac * scan_period_;
      }
      double tref = deskew_ref_end_ ? scan_period_ : 0.0;
      for (double &v:rel) v -= tref;
      static bool warned=false;
      if (!warned && enable_deskew_){
        warned=true;
        RCLCPP_WARN(get_logger(), "Deskew: no '%s' field. Using linear approx with scan_period=%.3f s.",
                    time_field_name_.c_str(), scan_period_);
      }
    }
    return rel;
  }

  // odom 插值查找（odom->base）
  std::optional<Eigen::Matrix4f> getOdomPoseAt(const rclcpp::Time& t){
    std::lock_guard<std::mutex> lk(odom_mtx_);
    if (odom_buf_.size()<2) return std::nullopt;
    if (t < odom_buf_.front().stamp || t > odom_buf_.back().stamp) return std::nullopt;
    size_t lo=0, hi=odom_buf_.size()-1;
    while (hi-lo>1){
      size_t mid=(lo+hi)/2;
      if (odom_buf_[mid].stamp <= t) lo=mid; else hi=mid;
    }
    const auto &A=odom_buf_[lo], &B=odom_buf_[hi];
    double dt=(B.stamp-A.stamp).seconds();
    if (dt<=1e-6) return A.T;
    double alpha=(t-A.stamp).seconds()/dt;
    return interpolateSE3(A.T, B.T, alpha);
  }

  // cloud 原地左乘 4x4（P := T * P）
  static void transformCloudInPlace(CloudT& cloud, const Eigen::Matrix4f& T){
    const Eigen::Matrix3f R = T.block<3,3>(0,0);
    const Eigen::Vector3f t = T.block<3,1>(0,3);
    for (auto &p : cloud){
      Eigen::Vector3f v(p.x, p.y, p.z);
      v = R * v + t;
      p.x = v.x(); p.y = v.y(); p.z = v.z();
    }
  }

  // --------- 订阅回调 ---------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    TimedPose tp{ rclcpp::Time(msg->header.stamp), poseMsgToMat4(msg->pose.pose) };
    {
      std::lock_guard<std::mutex> lk(odom_mtx_);
      odom_buf_.push_back(tp);
      const double keep_sec=8.0;
      while (!odom_buf_.empty() && (tp.stamp - odom_buf_.front().stamp).seconds() > keep_sec)
        odom_buf_.pop_front();
    }
    // 也存最新一条，作为后备
    std::lock_guard<std::mutex> lk2(odom_latest_mtx_);
    T_odom_latest_ = tp.T;
    have_odom_latest_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 读入点云（兼容无rgb）
    CloudT::Ptr cloud(new CloudT());
    bool had_rgb=false;
    if (!msgToCloudXYZRGB(*msg, cloud, had_rgb)) return;
    if (cloud->empty()) return;

    // 单位归一化
    const double s = decideUnitScale(*cloud);
    if (std::abs(s-1.0) > 1e-9){
      for (auto &p : *cloud){ p.x*=s; p.y*=s; p.z*=s; }
      if (debug_log_) RCLCPP_INFO(get_logger(), "Applied unit scale %.6f", s);
    }

    // per-point time & deskew（在传感器系）
    std::vector<double> rel_times;
    if (enable_deskew_) rel_times = extractPointTimes(*msg);
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);
    if (cloud->empty()) return;

    // 距离裁剪
    const double min_r2 = min_range_*min_range_, max_r2 = max_range_*max_range_;
    CloudT::Ptr ranged(new CloudT()); ranged->reserve(cloud->size());
    std::vector<double> times_ranged; times_ranged.reserve(cloud->size());
    for (size_t i=0;i<cloud->size();++i){
      const auto &p = (*cloud)[i];
      const double r2 = double(p.x)*p.x + double(p.y)*p.y + double(p.z)*p.z;
      if (std::isfinite(r2) && r2>=min_r2 && r2<=max_r2){
        ranged->push_back(p);
        if (enable_deskew_) times_ranged.push_back(i<rel_times.size()?rel_times[i]:0.0);
      }
    }
    if (ranged->empty()) return;

    // === 获取该帧的 odom 位姿（odom->base）并构造 map 位姿 ===
    // T_global_ 代表 (map->base) 的当前解；T_map_odom_ 是桥 (map->odom)
    Eigen::Matrix4f T_odom_now;
    bool have_now = false;
    if (auto T_ref = getOdomPoseAt(msg->header.stamp); T_ref.has_value()){
      T_odom_now = *T_ref; have_now = true;
    } else {
      std::lock_guard<std::mutex> lk2(odom_latest_mtx_);
      if (have_odom_latest_) { T_odom_now = T_odom_latest_; have_now = true; }
    }
    if (!have_now){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No odom for this cloud. Skip.");
      return;
    }

    // 首帧初始化：把第一帧的 odom 原点锚定到 map（T_map_odom_=I 即 map≡odom）
    if (!map_initialized_){
      // 体素
      pcl::VoxelGrid<PointT> vg;
      vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      vg.setInputCloud(ranged);
      CloudT::Ptr filtered(new CloudT());
      vg.filter(*filtered);
      if (filtered->empty()) return;

      T_map_odom_.setIdentity();        // map 与 odom 对齐
      T_global_ = T_map_odom_ * T_odom_now; // map->base
      last_fused_T_ = T_global_;

      // 把第一帧转到 map 系并存为地图
      CloudT::Ptr first_in_map(new CloudT(*filtered));
      transformCloudInPlace(*first_in_map, T_global_);
      map_cloud_ = first_in_map;
      map_initialized_ = true;
      fused_count_ = 1;

      publishAll(msg->header.stamp);
      RCLCPP_INFO(get_logger(), "Init map (pure-odom). pts=%zu (rgb=%d)", map_cloud_->size(), (int)had_rgb);
      return;
    }

    // deskew：在 odom 系下把每个点扭到参考时刻，然后再乘 map<-odom
    if (enable_deskew_) deskewInOdom(*ranged, times_ranged, msg->header.stamp);

    // 体素
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    vg.setInputCloud(ranged);
    CloudT::Ptr filtered(new CloudT());
    vg.filter(*filtered);
    if (filtered->empty()) return;

    // 计算当前帧的 map 位姿（map->base）
    T_global_ = T_map_odom_ * T_odom_now;

    // 关键帧融合（把本帧点云乘以 T_global_ 再叠加）
    if (shouldFuse(T_global_, last_fused_T_)){
      CloudT::Ptr in_map(new CloudT(*filtered));
      transformCloudInPlace(*in_map, T_global_);
      {
        std::lock_guard<std::mutex> lk(map_mutex_);
        *map_cloud_ += *in_map;
        ++fused_count_;
        if (global_voxel_every_n_<=1 || (fused_count_%global_voxel_every_n_==0)){
          vg.setInputCloud(map_cloud_);
          CloudT::Ptr down(new CloudT());
          vg.filter(*down);
          map_cloud_.swap(down);
        }
      }
      last_fused_T_ = T_global_;
    }

    publishAll(msg->header.stamp);
  }

  // deskew：把每点从Ti扭到Tref（这里在 odom 系内完成）
  void deskewInOdom(CloudT& cloud, const std::vector<double>& rel_times, const rclcpp::Time& frame_stamp){
    if (!enable_deskew_) return;
    if (cloud.size()!=rel_times.size()) return;
    auto T_ref_opt = getOdomPoseAt(frame_stamp);
    if (!T_ref_opt.has_value()){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Deskew skipped: no odom at frame time.");
      return;
    }
    const Eigen::Matrix4f T_ref = *T_ref_opt;
    for (size_t i=0;i<cloud.size();++i){
      rclcpp::Time ti = frame_stamp + rclcpp::Duration::from_seconds(rel_times[i]);
      auto T_i_opt = getOdomPoseAt(ti);
      if (!T_i_opt.has_value()) continue;
      const Eigen::Matrix4f T_i = *T_i_opt;
      Eigen::Matrix3f R = T_ref.block<3,3>(0,0) * T_i.block<3,3>(0,0).transpose();
      Eigen::Vector3f t = T_ref.block<3,1>(0,3) - R * T_i.block<3,1>(0,3);
      Eigen::Vector3f p(cloud[i].x, cloud[i].y, cloud[i].z);
      Eigen::Vector3f q = R * p + t;
      cloud[i].x = q.x(); cloud[i].y = q.y(); cloud[i].z = q.z();
    }
  }

  // 是否融合为关键帧
  bool shouldFuse(const Eigen::Matrix4f& T_now, const Eigen::Matrix4f& T_last){
    Eigen::Vector3f t_now = T_now.block<3,1>(0,3), t_last = T_last.block<3,1>(0,3);
    double trans = (t_now - t_last).norm();
    Eigen::Matrix3f R = T_last.block<3,3>(0,0).transpose() * T_now.block<3,3>(0,0);
    double ang = std::acos(std::min(1.0f,std::max(-1.0f,(R.trace()-1.0f)/2.0f)));
    double deg = ang * 180.0 / M_PI;
    return (trans >= kf_trans_thresh_) || (deg >= kf_rot_deg_thresh_);
  }

  // --------- 发布/保存 ---------
  void publishAll(const rclcpp::Time &stamp) {
    // 地图
    if (!map_cloud_ || map_cloud_->empty()) return;
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*map_cloud_, out);
    out.header.frame_id = output_frame_;
    out.header.stamp = stamp;
    map_pub_->publish(out);

    // 里程计（map->base）
    Eigen::Matrix3f R = T_global_.block<3,3>(0,0);
    Eigen::Vector3f t = T_global_.block<3,1>(0,3);
    Eigen::Quaternionf q(R);

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = output_frame_;
    odom.child_frame_id = "mapping_base";
    odom.header.stamp = stamp;
    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom);

    // 轨迹缓存（用于保存 CSV）
    traj_.push_back({stamp, t, q});
  }

  void onTimer(){
    std::lock_guard<std::mutex> lk(map_mutex_);
    if (!map_cloud_ || map_cloud_->empty()){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Auto-save skipped: map empty.");
      return;
    }
    (void)saveNowUnlocked();
  }

  void saveService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::lock_guard<std::mutex> lk(map_mutex_);
    res->success = saveNowUnlocked();
    res->message = res->success ? "Map saved." : "Save failed.";
  }

  bool saveNowUnlocked() {
    if (!map_cloud_ || map_cloud_->empty()){
      RCLCPP_WARN(get_logger(), "Map empty, skip save.");
      return false;
    }
    // 拷贝一份避免长时间持锁
    CloudT::Ptr map_copy(new CloudT(*map_cloud_));
    const rclcpp::Time now = this->now();

    // 文件名
    std::ostringstream oss;
    oss << "map";
    if (stamp_in_name_) {
      const int64_t sec  = now.seconds(); // floor 秒
      const int64_t nsec = now.nanoseconds() % 1000000000LL;
      oss << "_" << sec << "_" << std::setw(9) << std::setfill('0') << nsec;
    }
    const std::string base = oss.str();
    const std::string path = save_dir_ + "/" + base + (save_format_=="pcd" ? ".pcd" : ".ply");

    int ret = 0;
    if (save_format_=="pcd") ret = pcl::io::savePCDFileBinary(path, *map_copy);
    else                     ret = pcl::io::savePLYFileBinary(path, *map_copy);

    if (ret != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to save map to %s", path.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Saved map: %s (pts=%zu)", path.c_str(), map_copy->size());

    if (save_traj_csv_) {
      saveTrajectoryCSV(base);
    }
    return true;
  }

  void saveTrajectoryCSV(const std::string& base){
    if (traj_.empty()){
      RCLCPP_WARN(get_logger(), "No trajectory cached. Skip CSV.");
      return;
    }
    const std::string csv = save_dir_ + "/" + base + "_traj.csv";
    std::ofstream ofs(csv);
    if (!ofs.is_open()){
      RCLCPP_ERROR(get_logger(), "Cannot open %s", csv.c_str());
      return;
    }
    ofs << "sec,nsec,x,y,z,qx,qy,qz,qw\n";
    for (const auto& e : traj_){
      const int64_t sec  = e.stamp.seconds();
      const int64_t nsec = e.stamp.nanoseconds() % 1000000000LL;
      ofs << sec << "," << nsec << ","
          << e.t.x() << "," << e.t.y() << "," << e.t.z() << ","
          << e.q.x() << "," << e.q.y() << "," << e.q.z() << "," << e.q.w() << "\n";
    }
    ofs.close();
    RCLCPP_INFO(get_logger(), "Saved trajectory CSV: %s (rows=%zu)", csv.c_str(), traj_.size());
  }

  // --------- 成员 ---------
  // 参数
  std::string input_topic_, output_frame_, save_dir_, save_format_, time_field_name_;
  bool   enable_deskew_, deskew_ref_end_, auto_unit_scale_, debug_log_;
  bool   stamp_in_name_, save_traj_csv_;
  int    global_voxel_every_n_;
  double leaf_size_, min_range_, max_range_;
  double scan_period_, unit_scale_param_;
  double kf_trans_thresh_, kf_rot_deg_thresh_;
  double save_period_sec_;

  // ROS I/O
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 地图/位姿
  std::mutex map_mutex_;
  pcl::PointCloud<PointT>::Ptr map_cloud_{ new pcl::PointCloud<PointT>() };
  bool map_initialized_{false};
  Eigen::Matrix4f T_global_{Eigen::Matrix4f::Identity()};     // map->base
  Eigen::Matrix4f last_fused_T_{Eigen::Matrix4f::Identity()};
  size_t fused_count_{0};
  Eigen::Matrix4f T_map_odom_{Eigen::Matrix4f::Identity()};   // map->odom

  // 里程计缓存
  std::mutex odom_mtx_;
  std::deque<TimedPose> odom_buf_;
  std::mutex odom_latest_mtx_;
  Eigen::Matrix4f T_odom_latest_{Eigen::Matrix4f::Identity()};
  bool have_odom_latest_{false};

  // 轨迹缓存（用于CSV）
  std::vector<TrajEntry> traj_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  return 0;
}
