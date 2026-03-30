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

#include <plain_slam/normal_map.hpp>

namespace pslam {

NormalMap::NormalMap() {
  const size_t max_keyframe_size = 20;
  keyframe_poses_ = boost::circular_buffer<Sophus::SE3f>(max_keyframe_size);
  aligned_scan_clouds_ = boost::circular_buffer<PointCloud3f>(max_keyframe_size);

  filter_size_ = 0.1f;
  normal_eigen_val_thresh_ = 0.2f;
  num_normal_points_ = 10;
}

NormalMap::~NormalMap() {

}

void NormalMap::AddKeyframe(
  const Sophus::SE3f& keyframe_pose,
  const PointCloud3f& aligned_scan_cloud) {
  keyframe_poses_.push_back(keyframe_pose);
  aligned_scan_clouds_.push_back(aligned_scan_cloud);
  const size_t N = std::min(aligned_scan_clouds_.size(), keyframe_poses_.size());

  PointCloud3f merged_cloud;
  merged_cloud.reserve(1000000);
  for (size_t i = 0; i < N; ++i) {
    for (const auto& p: aligned_scan_clouds_[i]) {
      merged_cloud.push_back(p);
    }
  }

  const VoxelGridFilter vgf(filter_size_);
  filtered_map_cloud_ = vgf.filter(merged_cloud);

  normal_computed_flags_.resize(filtered_map_cloud_.size());
  normal_valid_flags_.resize(filtered_map_cloud_.size());
  normals_.resize(filtered_map_cloud_.size());
  for (size_t i = 0; i < filtered_map_cloud_.size(); ++i) {
    normal_computed_flags_[i] = false;
    normal_valid_flags_[i] = true;
  }

  adaptor_ = std::make_unique<PointCloudAdaptor>(filtered_map_cloud_);
  kdtree_ = std::make_unique<KDTree>(3, *adaptor_,
    nanoflann::KDTreeSingleIndexAdaptorParams(10));
  kdtree_->buildIndex();
}

bool NormalMap::FindCorrespondence(
  const Eigen::Vector3f& query,
  float max_correspondence_dist,
  Eigen::Vector3f& target,
  Eigen::Vector3f& normal) {
  if (!kdtree_ || !adaptor_) {
    return false;
  }

  const float dist2 = max_correspondence_dist * max_correspondence_dist;
  std::vector<size_t> nn_idx(1);
  std::vector<float> nn_dist(1);
  nanoflann::KNNResultSet<float> nn_result_set(1);
  nn_result_set.init(nn_idx.data(), nn_dist.data());
  kdtree_->findNeighbors(nn_result_set,
    query.data(), nanoflann::SearchParameters());
  if (nn_dist[0] > dist2) {
    return false;
  }

  const size_t idx = nn_idx[0];
  if (!normal_valid_flags_[idx]) {
    return false;
  }

  target = filtered_map_cloud_[idx];

  if (!normal_computed_flags_[idx]) {
    normal_computed_flags_[idx] = true;

    std::vector<size_t> indices(num_normal_points_);
    std::vector<float> dists(num_normal_points_);
    nanoflann::KNNResultSet<float> result_set(num_normal_points_);
    result_set.init(indices.data(), dists.data());
    kdtree_->findNeighbors(result_set, query.data(), nanoflann::SearchParameters());

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < num_normal_points_; ++i) {
      mean += filtered_map_cloud_[indices[i]];
    }
    mean /= static_cast<float>(num_normal_points_);

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < num_normal_points_; ++i) {
      const Eigen::Vector3f d = filtered_map_cloud_[indices[i]] - mean;
      cov += d * d.transpose();
    }
    cov += 1e-3f * Eigen::Matrix3f::Identity();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) {
      normal_valid_flags_[idx] = false;
      return false;
    }

    int min_idx;
    float val = solver.eigenvalues().minCoeff(&min_idx);
    const Eigen::Vector3f evals = solver.eigenvalues();
    const float lambda0 = evals(0);
    const float lambda1 = evals(1);
    const float lambda2 = evals(2);
    val = val / (lambda0 + lambda1 + lambda2);
    if (val > normal_eigen_val_thresh_) {
      normal_valid_flags_[idx] = false;
      return false;
    }

    normal = solver.eigenvectors().col(min_idx).normalized();
    // if (normal.dot(query - mean) > 0) {
    //   normal = -normal;
    // }

    normals_[idx] = normal;
  } else {
    normal = normals_[idx];
  }

  return true;
}

void NormalMap::GetActiveMapCloud(PointCloud3f& cloud) const {
  if (filtered_map_cloud_.size() == 0) {
    return;
  }

  size_t count = 0;
  for (size_t i = 0; i < filtered_map_cloud_.size(); ++i) {
    if (normal_computed_flags_[i] && normal_valid_flags_[i]) {
      count++;
    }
  }

  if (count == 0) {
    return;
  }

  cloud.reserve(count);

  for (size_t i = 0; i < filtered_map_cloud_.size(); ++i) {
    if (normal_computed_flags_[i] && normal_valid_flags_[i]) {
      cloud.emplace_back(filtered_map_cloud_[i]);
    }
  }
}

} // namespace pslam
