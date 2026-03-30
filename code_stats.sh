#!/bin/bash

# echo "========== plain_slam_ros2 total code information =========="
# cloc src include/plain_slam include/plain_slam_ros2
# echo -e "\n\n"

clear

echo -e "\n\n\n"

echo "===== plain_slam total C++ code information (excluding ROS2-related code) ====="
find src/ include/plain_slam/ -name "*.cpp" -o -name "*.hpp" \
  | grep -v "_node.cpp" \
  | grep -v "ros_utils*" \
  | grep -v "scan_intensity_matcher*" \
  | grep -v "lidar_imu_calibration*" \
  | grep -v "GNSSConvert*" \
  | grep -v "odom_gnss_recorder*" \
  | grep -v "tgm_estimator*" \
  | xargs cloc

echo -e "\n\n\n"
