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

#include <sophus/se3.hpp>

#include <plain_slam/types.hpp>
#include <plain_slam/voxel_grid_filter.hpp>
#include <plain_slam/lie_algebra.hpp>

namespace pslam {

class GICP {
 public:
  GICP();

  ~GICP();

  void SetFilterSize(float size) {
    filter_size_ = size;
  }

  void SetMaxSourcePoints(size_t num) {
    num_max_source_points_ = num;
  }

  void SetMaxIteration(size_t num) {
    num_max_iteration_ = num;
  }

  void SetEpsilon(float epsilon) {
    epsilon_ = epsilon;
  }

  void SetNumCovPoints(size_t num) {
    num_cov_points_ = num;
  }

  void SetMaxCorrespondenceDist(float dist) {
    max_correspondence_dist_ = dist;
    max_correspondence_dist2_ = dist * dist;
  }

  void SetHuberDelta(float delta) {
    huber_delta_ = delta;
  }

  bool HasConverged() const {
    return has_converged_;
  }

  float GetHuberDelta() const {
    return huber_delta_;
  }

  Sophus::SE3f GetTransformation() const {
    return T_;
  }

  float GetActivePointsRate() const {
    return active_points_rate_;
  }

  float GetErrorAverage() const {
    return error_average_;
  }

  void SetTransformation(
    const Eigen::Matrix3f& R,
    const Eigen::Vector3f& t) {
    T_ = Sophus::SE3f(R, t);
  }

  void SetTransformation(const Sophus::SE3f& T) {
    T_ = T;
  }

  void SetSourceCloud(
    const PointCloud3f& cloud,
    bool filter_cloud);

  void SetTargetCloud(
    const PointCloud3f& cloud,
    bool filter_cloud);

  void Align();

  void Transform(
    const PointCloud3f& in,
    PointCloud3f& out);

 private:
  Sophus::SE3f T_;
  float filter_size_;
  size_t num_max_source_points_;
  size_t num_max_iteration_;
  float epsilon_;
  size_t num_cov_points_;
  float max_correspondence_dist_;
  float max_correspondence_dist2_;
  float huber_delta_;

  bool has_converged_;
  float active_points_rate_;
  float error_average_;

  PointCloud3f source_points_;
  PointCloud3f source_means_;
  Covariances3f source_covs_;
  std::unique_ptr<PointCloudAdaptor> source_adaptor_;
  std::unique_ptr<KDTree> source_kdtree_;
  std::vector<bool> source_point_computed_flags_;

  PointCloud3f target_points_;
  PointCloud3f target_means_;
  Covariances3f target_covs_;
  std::unique_ptr<PointCloudAdaptor> target_adaptor_;
  std::unique_ptr<KDTree> target_kdtree_;
  std::vector<bool> target_point_computed_flags_;

  void ComputeGICPPoint(
    const Point3f& query,
    const KDTree& kdtree,
    const PointCloudAdaptor& adaptor,
    Point3f& mean,
    Covariance3f& cov);
};

} // namespace pslam
