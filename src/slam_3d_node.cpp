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
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <chrono>

#include <plain_slam/slam_3d_interface.hpp>

#include <plain_slam_ros2/ros_utils.hpp>

class SLAM3DNode : public rclcpp::Node {
 public:
  SLAM3DNode()
  : Node("slam_3d_node") {
    this->declare_parameter<std::string>("map_cloud_dir", "/tmp/pslam_data/");
    std::string map_cloud_dir = "/tmp/pslam_data/";
    this->get_parameter("map_cloud_dir", map_cloud_dir);
    slam_.SetSLAMDataDir(map_cloud_dir);
    slam_.MakeSLAMDataDir();
    slam_.ClearSLAMDataDir();

    this->declare_parameter<std::string>("param_files_dir", "");
    std::string param_files_dir = "";
    this->get_parameter("param_files_dir", param_files_dir);
    slam_.SetParamFilesDir(param_files_dir);

    // The SLAM node receives the IMU pose and the deskewed scan cloud,
    // which are published by the LIO node.
    this->declare_parameter<std::string>("imu_pose_topic", "/pslam/imu_pose");
    this->declare_parameter<std::string>("deskewed_scan_cloud_topic", "/pslam/deskewed_scan_cloud");
    std::string imu_pose_topic;
    std::string deskewed_scan_cloud_topic;
    this->get_parameter("imu_pose_topic", imu_pose_topic);
    this->get_parameter("deskewed_scan_cloud_topic", deskewed_scan_cloud_topic);
    pose_sub_.subscribe(this, imu_pose_topic, rmw_qos_profile_sensor_data);
    cloud_sub_.subscribe(this, deskewed_scan_cloud_topic, rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<message_filters::TimeSynchronizer<
      geometry_msgs::msg::PoseStamped,
      sensor_msgs::msg::PointCloud2>>(pose_sub_, cloud_sub_, 10);

    sync_->registerCallback(
      std::bind(&SLAM3DNode::SyncCallback, this, std::placeholders::_1, std::placeholders::_2));

    // These published topics are used for visualization in RViz.
    this->declare_parameter<std::string>("filtered_map_cloud_topic", "/pslam/filtered_map_cloud");
    std::string filtered_map_cloud_topic;
    this->get_parameter("filtered_map_cloud_topic", filtered_map_cloud_topic);
    filtered_map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_map_cloud_topic, rclcpp::QoS(1).transient_local().reliable());

    this->declare_parameter<std::string>("graph_nodes_topic", "/pslam/pose_graph_nodes");
    std::string graph_nodes_topic;
    this->get_parameter("graph_nodes_topic", graph_nodes_topic);
    graph_nodes_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      graph_nodes_topic, 1);

    this->declare_parameter<std::string>("odom_edges_topic", "/pslam/pose_graph_odom_edges");
    std::string odom_edges_topic;
    this->get_parameter("odom_edges_topic", odom_edges_topic);
    odom_edges_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      odom_edges_topic, 1);

    this->declare_parameter<std::string>("loop_edges_topic", "/pslam/pose_graph_loop_edges");
    std::string loop_edges_topic;
    this->get_parameter("loop_edges_topic", loop_edges_topic);
    loop_edges_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      loop_edges_topic, 1);

    this->declare_parameter<std::string>("graph_poses_topic", "/pslam/pose_graph_poses");
    std::string graph_poses_topic;
    this->get_parameter("graph_poses_topic", graph_poses_topic);
    graph_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      graph_poses_topic, 1);

    // The SLAM node broadcasts a transformation from map_frame_ to odom_frame_.
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("imu_frame", "imu");
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("imu_frame", imu_frame_);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    slam_.ReadSLAMParams();

    RCLCPP_INFO(this->get_logger(), "SLAM 3D node initialized");
  }

  ~SLAM3DNode() {
    slam_.BuildMapCloud();
    slam_.WriteMapCloudPCD();
  }

 private:

  void SyncCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
    const Eigen::Vector3f odom_trans = Eigen::Vector3f(pose->pose.position.x,
      pose->pose.position.y, pose->pose.position.z);
    const Eigen::Quaternionf odom_quat = Eigen::Quaternionf(pose->pose.orientation.w,
      pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
    const Sophus::SE3f odom_pose(odom_quat.toRotationMatrix(), odom_trans);

    pslam::PointCloud3f scan_cloud;
    std::vector<float> scan_intensities;
    std::vector<double> scan_stamps;
    ParsePSLAMCloud(cloud, scan_cloud, scan_intensities, scan_stamps);
    if (scan_cloud.size() == 0) {
      std::cout << "Skipping SLAM process because the scan cloud is empty." << std::endl;
      return;
    }

    // const auto t1 = std::chrono::high_resolution_clock::now();

    // Main process of the SLAM system
    slam_.SetData(odom_pose, scan_cloud, scan_intensities);

    // const auto t2 = std::chrono::high_resolution_clock::now();
    // std::cout << "elapsed time [msec]: " 
    //   << std::chrono::duration_cast<std::chrono::milliseconds>
    //        (t2 - t1).count() << std::endl;

    const Sophus::SE3f slam_pose = slam_.GetSLAMPose();
    const Sophus::SE3f T_map_odom = slam_pose * odom_pose.inverse();
    BroadcastTransform(tf_broadcaster_, map_frame_, odom_frame_,
      pose->header.stamp, T_map_odom);

    // Publish the map and pose graph data
    if (slam_.IsPoseGraphUpdated()) {
      const pslam::PointCloud3f filtered_map_cloud = slam_.GetFilteredMapCloud();
      PublishPointCloud(filtered_map_cloud_pub_, map_frame_,
        pose->header.stamp, filtered_map_cloud);

      pslam::PointCloud3f graph_nodes;
      slam_.GetGraphNodes(graph_nodes);
      PublishPointMarkers(graph_nodes_pub_, map_frame_, pose->header.stamp,
        "graph_nodes", 0, 0.5f, 0.0f, 0.0f, 1.0f, 1.0f, graph_nodes);

      pslam::PointCloud3f odom_edges_start_points, odom_edges_end_points;
      slam_.GetOdomEdgePoints(odom_edges_start_points, odom_edges_end_points);
      PublishLineMarkers(odom_edges_pub_, map_frame_, pose->header.stamp,
        "odom_edges", 1, 0.1f, 1.0f, 1.0f, 1.0f, 1.0f,
        odom_edges_start_points, odom_edges_end_points);

      pslam::PointCloud3f loop_edges_start_points, loop_edges_end_points;
      slam_.GetLoopEdgePoints(loop_edges_start_points, loop_edges_end_points);
      PublishLineMarkers(loop_edges_pub_, map_frame_, pose->header.stamp,
        "loop_edges", 2, 0.1f, 0.0f, 1.0f, 0.0f, 1.0f,
        loop_edges_start_points, loop_edges_end_points);

      const std::vector<Sophus::SE3f> graph_poses = slam_.GetPoseGraph();
      PublishPoseArray(graph_poses_pub_, map_frame_, pose->header.stamp, graph_poses);

      slam_.SetPoseGraphUpdated(false);
     }
  }

  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<
    geometry_msgs::msg::PoseStamped,
    sensor_msgs::msg::PointCloud2>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_map_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr odom_edges_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_edges_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr graph_poses_pub_;

  pslam::SLAM3DInterface slam_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string imu_frame_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAM3DNode>());
  rclcpp::shutdown();
  return 0;
}