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

#pragma once

#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <plain_slam/types.hpp>

void ParseLivoxCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  pslam::PointCloud3f& scan_cloud,
  std::vector<float>& scan_intensities,
  std::vector<double>& scan_stamps);

void ParseOusterCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  double first_stamp,
  pslam::PointCloud3f& scan_cloud,
  std::vector<float>& scan_intensities,
  std::vector<double>& scan_stamps);

void ParsePSLAMCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
  pslam::PointCloud3f& scan_cloud,
  std::vector<float>& scan_intensities,
  std::vector<double>& scan_stamps);

void ParsePSLAMCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  pslam::PointCloud3f& scan_cloud);

void PublishPose(
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const Sophus::SE3f& T);

void PublishPoseArray(
  const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const std::vector<Sophus::SE3f>& Ts);

void PublishOdometry(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub,
  const std::string& frame_id,
  const std::string& child_frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const pslam::State& state,
  const pslam::StateCov& state_cov,
  const pslam::IMUMeasure& measure);

void BroadcastTransform(
  const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
  const std::string& parent_frame,
  const std::string& child_frame,
  const builtin_interfaces::msg::Time& stamp,
  const Sophus::SE3f& T);

void PublishPointCloud(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const pslam::PointCloud3f& cloud);
  
void PublishPointCloud(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const pslam::PointCloud3f& cloud,
  const std::vector<float>& scan_intensities,
  const std::vector<double>& scan_stamps);

void PublishPointMarkers(
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& ns,
  int id,
  float scale,
  float r,
  float g,
  float b,
  float a,
  const pslam::PointCloud3f& nodes);

void PublishLineMarkers(
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& ns,
  int id,
  float line_width,
  float r,
  float g,
  float b,
  float a,
  const pslam::PointCloud3f& start_points,
  const pslam::PointCloud3f& end_points);
