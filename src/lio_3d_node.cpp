/*
 * plain_slam_ros2
 * 
 * Copyright (c) 2025 Naoki Akai
 * All rights reserved.
 *
 * This software is provided free of charge for academic and personal use only.
 * Commercial use is strictly prohibited without prior written permission from the author.
 *
 * Conditions:
 *   - Non-commercial use only
 *   - Attribution required in any academic or derivative work
 *   - Redistribution permitted with this license header intact
 *
 * Disclaimer:
 *   This software is provided "as is" without warranty of any kind.
 *   The author is not liable for any damages arising from its use.
 *
 * For commercial licensing inquiries, please contact:
 *   Naoki Akai
 *   Email: n.akai.goo[at]gmail.com   ([at] -> @)
 *   Subject: [plain_slam_ros2] Commercial License Inquiry
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>

#include <plain_slam/lio_3d_interface.hpp>

#include <plain_slam_ros2/ros_utils.hpp>

class LIO3DNode : public rclcpp::Node {
 public:
  LIO3DNode()
  : Node("lio_3d_node") {
    // Get the LiDAR type to parse PointCloud2 messages
    this->declare_parameter<std::string>("lidar_type", "livox");    
    this->get_parameter("lidar_type", lidar_type_);
    if (lidar_type_ == "livox") {
      RCLCPP_INFO(this->get_logger(), "LiDAR type: Livox");
    } else if (lidar_type_ == "ouster") {
      RCLCPP_INFO(this->get_logger(), "LiDAR type: Ouster");
    } else {
      std::cout << "LiDAR type: Unknown (" << lidar_type_ << ")" << std::endl;
      RCLCPP_WARN(this->get_logger(),
        "Unknown LiDAR type '%s'.\n"
        "The point cloud will be parsed assuming the following fields:\n"
        "- x, y, z: float32\n"
        "- intensity: float32\n"
        "- timestamp: float64 (in seconds)\n",
        lidar_type_.c_str());
    }

    this->declare_parameter<bool>("use_as_localizer", false);
    bool use_as_localizer = false;
    this->get_parameter("use_as_localizer", use_as_localizer);
    if (use_as_localizer) {
      std::cout << "LIO starts in localization mode (map-based pose tracking)" << std::endl;
      lio_.SetLocalizationMode(true);
    } else {
      std::cout << "LIO starts in odometry mode (incremental pose estimation)" << std::endl;
      lio_.SetLocalizationMode(false);
    }

    this->declare_parameter<std::string>("param_files_dir", "");
    std::string param_files_dir = "";
    this->get_parameter("param_files_dir", param_files_dir);
    lio_.SetParamFilesDir(param_files_dir);
    RCLCPP_INFO(this->get_logger(), "LIO parameters dir: %s", param_files_dir.c_str());

    this->declare_parameter<std::string>("pointcloud_topic", "/livox/lidar");
    std::string pointcloud_topic;
    this->get_parameter("pointcloud_topic", pointcloud_topic);
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&LIO3DNode::PointcloudCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("imu_topic", "/livox/imu");
    std::string imu_topic;
    this->get_parameter("imu_topic", imu_topic);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS().keep_last(10000),
      std::bind(&LIO3DNode::ImuCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("imu_pose_topic", "/pslam/imu_pose");
    std::string imu_pose_topic;
    this->get_parameter("imu_pose_topic", imu_pose_topic);
    imu_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      imu_pose_topic, 10);

    this->declare_parameter<std::string>("imu_odom_topic", "/pslam/imu_odom");
    std::string imu_odom_topic;
    this->get_parameter("imu_odom_topic", imu_odom_topic);
    imu_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      imu_odom_topic, 100);

    this->declare_parameter<std::string>("lio_map_cloud_topic", "/pslam/lio_map_cloud");
    std::string lio_map_cloud_topic;
    this->get_parameter("lio_map_cloud_topic", lio_map_cloud_topic);
    lio_map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      lio_map_cloud_topic, rclcpp::QoS(1).transient_local().reliable());

    this->declare_parameter<std::string>("aligned_scan_cloud_topic", "/pslam/aligned_scan_cloud");
    std::string aligned_scan_cloud_topic;
    this->get_parameter("aligned_scan_cloud_topic", aligned_scan_cloud_topic);
    aligned_scan_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      aligned_scan_cloud_topic, 10);

    this->declare_parameter<std::string>("deskewed_scan_cloud_topic", "/pslam/deskewed_scan_cloud");
    std::string deskewed_scan_cloud_topic;
    this->get_parameter("deskewed_scan_cloud_topic", deskewed_scan_cloud_topic);
    deskewed_scan_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      deskewed_scan_cloud_topic, 10);

    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("imu_frame", "imu");
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("imu_frame", imu_frame_);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // This node broadcasts a transformation from odom_frame_ to imu_frame_,
    // even when operating in localization mode.
    // Please choose appropriate frame names.
    if (use_as_localizer) {
      this->declare_parameter<std::string>("map_cloud_dir", "/tmp/pslam_data/");
      std::string map_cloud_dir;
      this->get_parameter("map_cloud_dir", map_cloud_dir);
      if (!lio_.ReadMapCloudPCD(map_cloud_dir)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read map cloud from: %s",
          map_cloud_dir.c_str());        
        exit(1);
      }

      const pslam::PointCloud3f lio_map_cloud = lio_.GetNormalMapCloud();
      PublishPointCloud(lio_map_cloud_pub_,
        odom_frame_, rclcpp::Clock().now(), lio_map_cloud);
      lio_.SetMapUpdated(false);
    }

    lio_.ReadLIOParams();

    RCLCPP_INFO(this->get_logger(), "LIO 3D node initialized");
  }

 private:
  void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Parse the PointCloud2 message
    pslam::PointCloud3f scan_cloud;
    std::vector<float> scan_intensities;
    std::vector<double> scan_stamps;
    if (lidar_type_ == "livox") {
      ParseLivoxCloud(msg, scan_cloud, scan_intensities, scan_stamps);
    } else if (lidar_type_ == "ouster") {
      ParseOusterCloud(msg, rclcpp::Time(msg->header.stamp).seconds(),
        scan_cloud, scan_intensities, scan_stamps);
    } else {
      ParsePSLAMCloud(msg, scan_cloud, scan_intensities, scan_stamps);
    }

    // const auto t1 = std::chrono::high_resolution_clock::now();

    // The main process of LIO
    lio_.SetScanCloud(scan_cloud, scan_intensities, scan_stamps);

    // const auto t2 = std::chrono::high_resolution_clock::now();
    // std::cout << "elapsed time [msec]: " 
    //   << std::chrono::duration_cast<std::chrono::milliseconds>
    //   (t2 - t1).count() << std::endl;

    // Publish ROS messages
    const Sophus::SE3f imu_pose = lio_.GetIMUPose();
    PublishPose(imu_pose_pub_, odom_frame_, msg->header.stamp, imu_pose);

    BroadcastTransform(tf_broadcaster_, odom_frame_, imu_frame_,
      msg->header.stamp, imu_pose);

    const pslam::PointCloud3f aligned_scan_cloud = lio_.GetAlignedScanCloud();
    const std::vector<float> clipped_scan_intensities = lio_.GetClippedScanIntensities();
    const std::vector<double> clipped_scan_stamps = lio_.GetClippedScanStamps();
    PublishPointCloud(aligned_scan_cloud_pub_, odom_frame_, msg->header.stamp,
      aligned_scan_cloud, clipped_scan_intensities, clipped_scan_stamps);

    const pslam::PointCloud3f deskewed_scan_cloud = lio_.GetDeskewedScanCloud();
    PublishPointCloud(deskewed_scan_cloud_pub_, imu_frame_, msg->header.stamp,
      deskewed_scan_cloud, clipped_scan_intensities, clipped_scan_stamps);

    if (lio_.IsMapUpdated()) {
      const pslam::PointCloud3f lio_map_cloud = lio_.GetNormalMapCloud();
      PublishPointCloud(lio_map_cloud_pub_, odom_frame_, msg->header.stamp, lio_map_cloud);
      lio_.SetMapUpdated(false);
    }
  }

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    static bool is_first = true;
    static double prev_stamp;

    const double stamp = rclcpp::Time(msg->header.stamp).seconds();

    // Compute the delta time, as it is used for IMU preintegration.
    double dt;
    if (is_first) {
      prev_stamp = stamp;
      dt = 1.0 / 200.0;
      is_first = false;
    } else {
      dt = stamp - prev_stamp;
      prev_stamp = stamp;
    }

    pslam::IMUMeasure measure;
    measure.acc = Eigen::Vector3f(msg->linear_acceleration.x,
      msg->linear_acceleration.y, msg->linear_acceleration.z);
    measure.gyro = Eigen::Vector3f(msg->angular_velocity.x,
      msg->angular_velocity.y, msg->angular_velocity.z);
    measure.stamp = stamp;
    measure.dt = dt;

    // SetIMUMeasure only stores IMU data. The relevant IMU measurements
    // corresponding to the LiDAR data are extracted in SetScanCloud.
    // The acceleration scale is modified in SetIMUMeasure.
    lio_.SetIMUMeasure(measure);

    // Publish odometry
    pslam::State imu_odom_state;
    pslam::StateCov imu_odom_state_cov;
    lio_.GetIMUOdometry(imu_odom_state, imu_odom_state_cov);
    PublishOdometry(imu_odom_pub_, odom_frame_, imu_frame_, msg->header.stamp,
      imu_odom_state, imu_odom_state_cov, measure);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lio_map_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_scan_cloud_pub_;

  std::string lidar_type_;

  pslam::LIO3DInterface lio_;

  std::string odom_frame_;
  std::string imu_frame_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LIO3DNode>());
  rclcpp::shutdown();
  return 0;
}
