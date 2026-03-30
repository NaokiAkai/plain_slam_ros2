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

#include <plain_slam_ros2/ros_utils.hpp>

void ParseLivoxCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  pslam::PointCloud3f& scan_cloud,
  std::vector<float>& scan_intensities,
  std::vector<double>& scan_stamps)
{
  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int intensity_offset = -1;
  int timestamp_offset = -1;

  for (const auto& field : msg->fields) {
    if (field.name == "x") x_offset = field.offset;
    else if (field.name == "y") y_offset = field.offset;
    else if (field.name == "z") z_offset = field.offset;
    else if (field.name == "intensity") intensity_offset = field.offset;
    else if (field.name == "timestamp") timestamp_offset = field.offset;
  }

  const size_t point_step = msg->point_step;
  const size_t point_count = msg->width * msg->height;
  const auto& data = msg->data;

  scan_cloud.resize(point_count);
  scan_intensities.resize(point_count);
  scan_stamps.resize(point_count);

  for (size_t i = 0; i < point_count; ++i) {
    const uint8_t* point_ptr = &data[i * point_step];

    pslam::Point3f pt;
    std::memcpy(&pt.x(), point_ptr + x_offset, sizeof(float));
    std::memcpy(&pt.y(), point_ptr + y_offset, sizeof(float));
    std::memcpy(&pt.z(), point_ptr + z_offset, sizeof(float));
    scan_cloud[i] = pt;

    std::memcpy(&scan_intensities[i], point_ptr + intensity_offset, sizeof(float));

    std::memcpy(&scan_stamps[i], point_ptr + timestamp_offset, sizeof(double));
    scan_stamps[i] *= 1e-9;
    // printf("scan_stamps[%lu] = %.10lf\n", i, scan_stamps[i]);
  }
}

void ParseOusterCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  double first_stamp,
  pslam::PointCloud3f& scan_cloud,
  std::vector<float>& scan_intensities,
  std::vector<double>& scan_stamps)
{
  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int intensity_offset = -1;
  int timestamp_offset = -1;

  for (const auto& field : msg->fields) {
    if (field.name == "x") x_offset = field.offset;
    else if (field.name == "y") y_offset = field.offset;
    else if (field.name == "z") z_offset = field.offset;
    else if (field.name == "intensity") intensity_offset = field.offset;
    else if (field.name == "t") timestamp_offset = field.offset;
  }

  const size_t point_step = msg->point_step;
  const size_t point_count = msg->width * msg->height;
  const auto& data = msg->data;

  scan_cloud.resize(point_count);
  scan_intensities.resize(point_count);
  scan_stamps.resize(point_count);

  for (size_t i = 0; i < point_count; ++i) {
    const uint8_t* point_ptr = &data[i * point_step];

    pslam::Point3f pt;
    std::memcpy(&pt.x(), point_ptr + x_offset, sizeof(float));
    std::memcpy(&pt.y(), point_ptr + y_offset, sizeof(float));
    std::memcpy(&pt.z(), point_ptr + z_offset, sizeof(float));
    scan_cloud[i] = pt;

    std::memcpy(&scan_intensities[i], point_ptr + intensity_offset, sizeof(float));

    uint32_t t;
    std::memcpy(&t, point_ptr + timestamp_offset, sizeof(uint32_t));
    scan_stamps[i] = first_stamp + static_cast<double>(t) * 1e-9;
    // printf("orig = %d, scan_stamps[%lu] = %.10lf\n", t, i, scan_stamps[i]);
  }
}

void ParsePSLAMCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
  pslam::PointCloud3f& scan_cloud,
  std::vector<float>& scan_intensities,
  std::vector<double>& scan_stamps)
{
  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int intensity_offset = -1;
  int timestamp_offset = -1;

  for (const auto& field : msg->fields) {
    if (field.name == "x") x_offset = field.offset;
    else if (field.name == "y") y_offset = field.offset;
    else if (field.name == "z") z_offset = field.offset;
    else if (field.name == "intensity") intensity_offset = field.offset;
    else if (field.name == "timestamp") timestamp_offset = field.offset;
  }

  const size_t point_step = msg->point_step;
  const size_t point_count = msg->width * msg->height;
  const auto& data = msg->data;

  scan_cloud.resize(point_count);
  scan_intensities.resize(point_count);
  scan_stamps.resize(point_count);

  for (size_t i = 0; i < point_count; ++i) {
    const uint8_t* point_ptr = &data[i * point_step];

    pslam::Point3f pt;
    std::memcpy(&pt.x(), point_ptr + x_offset, sizeof(float));
    std::memcpy(&pt.y(), point_ptr + y_offset, sizeof(float));
    std::memcpy(&pt.z(), point_ptr + z_offset, sizeof(float));
    scan_cloud[i] = pt;

    std::memcpy(&scan_intensities[i], point_ptr + intensity_offset, sizeof(float));

    std::memcpy(&scan_stamps[i], point_ptr + timestamp_offset, sizeof(double));
  }
}

void ParsePSLAMCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  pslam::PointCloud3f& scan_cloud)
{
  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;

  for (const auto& field : msg->fields) {
    if (field.name == "x") x_offset = field.offset;
    else if (field.name == "y") y_offset = field.offset;
    else if (field.name == "z") z_offset = field.offset;
  }

  const size_t point_step = msg->point_step;
  const size_t point_count = msg->width * msg->height;
  const auto& data = msg->data;

  scan_cloud.resize(point_count);

  for (size_t i = 0; i < point_count; ++i) {
    const uint8_t* point_ptr = &data[i * point_step];

    pslam::Point3f pt;
    std::memcpy(&pt.x(), point_ptr + x_offset, sizeof(float));
    std::memcpy(&pt.y(), point_ptr + y_offset, sizeof(float));
    std::memcpy(&pt.z(), point_ptr + z_offset, sizeof(float));
    scan_cloud[i] = pt;
  }
}

void PublishPose(
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const Sophus::SE3f& T)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  const Eigen::Vector3f t = T.translation();
  const Eigen::Quaternionf q = Eigen::Quaternionf(T.rotationMatrix());
  msg.pose.position.x = t.x();
  msg.pose.position.y = t.y();
  msg.pose.position.z = t.z();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  pub->publish(msg);
}

void PublishPoseArray(
  const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const std::vector<Sophus::SE3f>& Ts)
{
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.poses.reserve(Ts.size());

  for (const auto& T : Ts) {
    geometry_msgs::msg::Pose p;
    const Eigen::Vector3f t = T.translation();
    Eigen::Quaternionf q(T.rotationMatrix());
    q.normalize();
    p.position.x = t.x();
    p.position.y = t.y();
    p.position.z = t.z();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    msg.poses.push_back(p);
  }

  pub->publish(msg);
}

void PublishOdometry(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub,
  const std::string& frame_id,
  const std::string& child_frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const pslam::State& state,
  const pslam::StateCov& state_cov,
  const pslam::IMUMeasure& measure) {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;

  const Eigen::Vector3f t = state.T.translation();
  const Eigen::Quaternionf q(state.T.so3().matrix());

  msg.pose.pose.position.x = t.x();
  msg.pose.pose.position.y = t.y();
  msg.pose.pose.position.z = t.z();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      msg.pose.covariance[i * 6 + j] = state_cov(i, j);
    }
  }

  msg.twist.twist.linear.x = state.v.x();
  msg.twist.twist.linear.y = state.v.y();
  msg.twist.twist.linear.z = state.v.z();
  msg.twist.twist.angular.x = measure.gyro.x();
  msg.twist.twist.angular.y = measure.gyro.y();
  msg.twist.twist.angular.z = measure.gyro.z();

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      msg.twist.covariance[i * 6 + j] = state_cov(6 + i, 6 + j);
    }
  }

  pub->publish(msg);
}

void BroadcastTransform(
  const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
  const std::string& parent_frame,
  const std::string& child_frame,
  const builtin_interfaces::msg::Time& stamp,
  const Sophus::SE3f& T)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;

  const Eigen::Vector3f t = T.translation();
  const Eigen::Quaternionf q(T.rotationMatrix());

  transform_msg.transform.translation.x = t.x();
  transform_msg.transform.translation.y = t.y();
  transform_msg.transform.translation.z = t.z();
  transform_msg.transform.rotation.x = q.x();
  transform_msg.transform.rotation.y = q.y();
  transform_msg.transform.rotation.z = q.z();
  transform_msg.transform.rotation.w = q.w();

  tf_broadcaster->sendTransform(transform_msg);
}

void PublishPointCloud(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const pslam::PointCloud3f& cloud)
{
  sensor_msgs::msg::PointCloud2 msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = cloud.size();
  msg.is_dense = false;
  msg.is_bigendian = false;
  msg.point_step = 3 * sizeof(float);
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;

  for (size_t i = 0; i < msg.width; ++i) {
    uint8_t* ptr = &msg.data[i * msg.point_step];
    float x = cloud[i].x();
    float y = cloud[i].y();
    float z = cloud[i].z();
    std::memcpy(ptr + 0,  &x, sizeof(float));
    std::memcpy(ptr + 4,  &y, sizeof(float));
    std::memcpy(ptr + 8,  &z, sizeof(float));
  }

  pub->publish(msg);
}

void PublishPointCloud(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const pslam::PointCloud3f& cloud,
  const std::vector<float>& scan_intensities,
  const std::vector<double>& scan_stamps)
{
  sensor_msgs::msg::PointCloud2 msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = cloud.size();
  msg.is_dense = false;
  msg.is_bigendian = false;
  msg.point_step = 4 * sizeof(float) + sizeof(double);
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  msg.fields.resize(5);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;

  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[3].count = 1;

  msg.fields[4].name = "timestamp";
  msg.fields[4].offset = 16;
  msg.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT64;
  msg.fields[4].count = 1;

  for (size_t i = 0; i < msg.width; ++i) {
    uint8_t* ptr = &msg.data[i * msg.point_step];
    float x = cloud[i].x();
    float y = cloud[i].y();
    float z = cloud[i].z();
    float intensity = scan_intensities[i];
    double timestamp = scan_stamps[i];

    std::memcpy(ptr + 0, &x, sizeof(float));
    std::memcpy(ptr + 4, &y, sizeof(float));
    std::memcpy(ptr + 8, &z, sizeof(float));
    std::memcpy(ptr + 12, &intensity, sizeof(float));
    std::memcpy(ptr + 16, &timestamp, sizeof(double));
  }

  pub->publish(msg);
}

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
  const pslam::PointCloud3f& nodes) {
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.ns = ns;
  msg.id = id;
  msg.type = visualization_msgs::msg::Marker::POINTS;
  msg.action = visualization_msgs::msg::Marker::ADD;

  msg.scale.x = scale;
  msg.scale.y = scale;
  msg.scale.z = scale;
  msg.color.r = r;
  msg.color.g = g;
  msg.color.b = b;
  msg.color.a = a;

  msg.points.resize(nodes.size());
  for (size_t i = 0; i < nodes.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = nodes[i].x();
    p.y = nodes[i].y();
    p.z = nodes[i].z();
    msg.points[i] = p;
  }

  pub->publish(msg);
}

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
  const pslam::PointCloud3f& end_points) {
  if (start_points.size() != end_points.size()) {
    std::cout << "[WARN] The number of line start and end points do not match." << std::endl;
    return;
  }

  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.ns = ns;
  msg.id = id;
  msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.scale.x = line_width;

  msg.color.r = r;
  msg.color.g = g;
  msg.color.b = b;
  msg.color.a = a;

  msg.points.resize(2 * start_points.size());
  for (size_t i = 0; i < start_points.size(); ++i) {
    geometry_msgs::msg::Point p;

    p.x = start_points[i].x();
    p.y = start_points[i].y();
    p.z = start_points[i].z();
    msg.points[2 * i] = p;

    p.x = end_points[i].x();
    p.y = end_points[i].y();
    p.z = end_points[i].z();
    msg.points[2 * i + 1] = p;
  }

  pub->publish(msg);
}
