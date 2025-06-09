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
 *   Email: n.akai.goo[at]gmail.com   ([at] → @)
 *   Subject: [plain_slam_ros2] Commercial License Inquiry
 */

#include <plain_slam/slam_3d_lib.hpp>

namespace pslam {

SLAM3DLib::SLAM3DLib() {
  slam_data_dir_ = "/tmp/pslam_data/";

  accumulation_cycle_ = 5;

  scan_cloud_filter_size_ = 0.25f;

  kf_detector_.SetDistanceThreshold(3.0f);
  kf_detector_.SetAngleThreshold(30.0f * M_PI / 180.0f);

  slam_pose_ = Sophus::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());

  graph_pose_points_.clear();

  max_num_loop_candidates_ = 10;
  loop_detection_distance_thresh_ = 30.0f;
  error_average_th_ = 100.0f;
  active_points_rate_th_ = 0.5f;

  is_odom_pose_initialized_ = false;
  is_pose_graph_updated_ = false;
}

SLAM3DLib::~SLAM3DLib() {
  
}

void SLAM3DLib::ReadSLAMParams() {
  const std::string yaml_file = param_files_dir_ + "/slam_3d_params.yaml";
  const YAML::Node config = YAML::LoadFile(yaml_file);

  accumulation_cycle_ = config["preprocess"]["accumulation_cycle"].as<size_t>();
  scan_cloud_filter_size_ = config["preprocess"]["scan_cloud_filter_size"].as<float>();

  const float keyframe_angle_th = config["keyframe"]["angle_th"].as<float>();
  kf_detector_.SetDistanceThreshold(config["keyframe"]["distance_th"].as<float>());
  kf_detector_.SetAngleThreshold(keyframe_angle_th * M_PI / 180.0f);

  max_num_loop_candidates_ = config["loop_closure"]["num_loop_candidates"].as<size_t>();
  loop_detection_distance_thresh_ = config["loop_closure"]["loop_detection_distance_thresh"].as<float>();
  error_average_th_ = config["loop_closure"]["error_average_th"].as<float>();
  active_points_rate_th_ = config["loop_closure"]["active_points_rate_th"].as<float>();
  gicp_.SetMaxIteration(config["loop_closure"]["gicp_max_iteration"].as<size_t>());
  gicp_.SetEpsilon(config["loop_closure"]["gicp_epsilon"].as<float>());
  gicp_.SetMaxCorrespondenceDist(config["loop_closure"]["gicp_max_correspondence_dist"].as<float>());
  gicp_.SetHuberDelta(config["loop_closure"]["gicp_huber_delta"].as<float>());

  g_optimizer_.SetHuberDelta(config["pose_graph"]["huber_delta"].as<float>());
}

void SLAM3DLib::SetData(
  const Sophus::SE3f& odom_pose,
  const PointCloud3f& scan_cloud,
  const std::vector<float>& scan_intensities) {
  if (!is_odom_pose_initialized_) {
    slam_pose_ = odom_pose;
    prev_odom_pose_ = odom_pose;
    is_odom_pose_initialized_ = true;
  }

  const Sophus::SE3f relative_odom_pose = prev_odom_pose_.inverse() * odom_pose;
  slam_pose_ = slam_pose_ * relative_odom_pose;
  prev_odom_pose_ = odom_pose;

  if (accumulated_count_ == 0) {
    accumulated_scan_cloud_.clear();
    accumulated_scan_intensities_.clear();
  }

  for (size_t i = 0; i < scan_cloud.size(); ++i) {
    const Point3f p = odom_pose * scan_cloud[i];
    accumulated_scan_cloud_.push_back(p);
    accumulated_scan_intensities_.push_back(scan_intensities[i]);
  }

  accumulated_count_++;
  if (accumulated_count_ != accumulation_cycle_) {
    return;
  }

  accumulated_count_ = 0;
  const Sophus::SE3f odom_pose_inv = odom_pose.inverse();
  for (size_t i = 0; i < accumulated_scan_cloud_.size(); ++i) {
    accumulated_scan_cloud_[i] = odom_pose_inv * accumulated_scan_cloud_[i];
  }

  const VoxelGridFilter vgf(scan_cloud_filter_size_);
  const PointCloud3f filtered_cloud = vgf.filter(accumulated_scan_cloud_);

  if (pose_graph_.size() == 0) {
    kf_detector_.AddKeyframe(slam_pose_);
    pose_graph_.emplace_back(slam_pose_);
    BuildGraphPoseKDTree();

    std::string raw_cloud_file, filtered_cloud_file;
    GetCloudFileNames(0, raw_cloud_file, filtered_cloud_file);
    WritePointCloud(raw_cloud_file, accumulated_scan_cloud_, accumulated_scan_intensities_);
    WritePointCloud(filtered_cloud_file, filtered_cloud);

    BuildFilteredMapCloud();

    is_pose_graph_updated_ = true;
    latest_keyframe_odom_pose_ = odom_pose;
    return;
  }

  if (!kf_detector_.IsKeyframe(slam_pose_)) {
    return;
  }

  is_pose_graph_updated_ = true;
  kf_detector_.UpdateKeyframe(slam_pose_);

  std::vector<size_t> target_indices;
  GetTargetCandidateIndices(slam_pose_, target_indices);
  pose_graph_.emplace_back(slam_pose_);

  std::string raw_cloud_file, filtered_cloud_file;
  GetCloudFileNames(pose_graph_.size() - 1, raw_cloud_file, filtered_cloud_file);
  WritePointCloud(raw_cloud_file, accumulated_scan_cloud_, accumulated_scan_intensities_);
  WritePointCloud(filtered_cloud_file, filtered_cloud);

  const size_t src_idx = pose_graph_.size() - 2;
  const size_t tgt_idx = pose_graph_.size() - 1;
  Edge odom_edge;
  odom_edge.source_idx = src_idx;
  odom_edge.target_idx = tgt_idx;
  odom_edge.relative = latest_keyframe_odom_pose_.inverse() * odom_pose;
  odom_edges_.emplace_back(odom_edge);
  latest_keyframe_odom_pose_ = odom_pose;

  gicp_.SetSourceCloud(filtered_cloud, false);
  const size_t prev_loop_edges_size = loop_edges_.size();

  for (size_t i = 0; i < target_indices.size(); ++i) {
    const Sophus::SE3f& target_pose = pose_graph_[target_indices[i]];
    const Eigen::Vector3f delta_trans = target_pose.translation() - slam_pose_.translation();
    const float delta = delta_trans.norm();
    if (delta > loop_detection_distance_thresh_) {
      continue;
    }

    std::string raw_cloud_file, filtered_cloud_file;
    GetCloudFileNames(target_indices[i], raw_cloud_file, filtered_cloud_file);
    PointCloud3f target_cloud;
    ReadPointCloud(filtered_cloud_file, target_cloud);
    TransformCloud(target_pose, target_cloud);
    gicp_.SetTargetCloud(target_cloud, false);
    gicp_.SetTransformation(slam_pose_);
    gicp_.Align();

    const bool has_converged = gicp_.HasConverged();
    const float error_average = gicp_.GetErrorAverage();
    const float active_points_rate = gicp_.GetActivePointsRate();
    std::cout << "For " << target_indices[i] << "th node" << std::endl;
    std::cout << "has_converged: " << has_converged << std::endl;
    std::cout << "error_average = " << error_average << std::endl;
    std::cout << "active_points_rate = " << active_points_rate << std::endl;

    if (has_converged && error_average < error_average_th_ &&
      active_points_rate > active_points_rate_th_) {
      const Sophus::SE3f source_pose = gicp_.GetTransformation();
      Edge loop_edge;
      loop_edge.source_idx = pose_graph_.size() - 1;
      loop_edge.target_idx = target_indices[i];
      loop_edge.relative = source_pose.inverse() * target_pose;
      loop_edges_.emplace_back(loop_edge);
    }
  }

  // Skip optimization if no new loop closures were detected
  const size_t new_loop_edges_size = loop_edges_.size();
  if (new_loop_edges_size == prev_loop_edges_size) {
    std::cout << "No loop closures were detected" << std::endl;
  } else {
    std::cout << new_loop_edges_size - prev_loop_edges_size << " edges were detected" << std::endl;
    g_optimizer_.Optimize(odom_edges_, loop_edges_, pose_graph_);
    slam_pose_ = pose_graph_.back();
  }

  BuildGraphPoseKDTree();
  BuildFilteredMapCloud();
}

void SLAM3DLib::MakeSLAMDataDir() {
  try {
    if (!std::filesystem::exists(slam_data_dir_)) {
      std::filesystem::create_directories(slam_data_dir_);
      std::cout << "[INFO] Created directory: " << slam_data_dir_ << std::endl;
    }
  } catch (const std::filesystem::filesystem_error& e) {
    std::cerr << "[ERROR] Failed to create directory: " << e.what() << std::endl;
  }
}

void SLAM3DLib::ClearSLAMDataDir() {
  try {
    for (const auto& entry : std::filesystem::directory_iterator(slam_data_dir_)) {
      if (std::filesystem::is_regular_file(entry.path())) {
        std::filesystem::remove(entry.path());
      }
    }
    std::cout << "[INFO] Removed all files in: " << slam_data_dir_ << std::endl;
  } catch (const std::filesystem::filesystem_error& e) {
    std::cerr << "[ERROR] " << e.what() << std::endl;
  }
}

void SLAM3DLib::BuildMapCloud() {
  map_cloud_.clear();
  map_cloud_.reserve(5000000);
  map_intensities_.reserve(5000000);
  for (int i = static_cast<int>(pose_graph_.size()) - 1; i >= 0; --i) {
    const Sophus::SE3f& T = pose_graph_[i];
    std::string raw_cloud_file, filtered_cloud_file;
    GetCloudFileNames(i, raw_cloud_file, filtered_cloud_file);
    PointCloud3f cloud;
    std::vector<float> intensities;
    ReadPointCloud(raw_cloud_file, cloud, intensities);
    for (size_t i = 0; i < cloud.size(); ++i) {
      map_cloud_.push_back(T * cloud[i]);
      map_intensities_.push_back(intensities[i]);
    }
  }
}

void SLAM3DLib::WriteMapCloudPCD() {
  if (map_cloud_.size() == 0) {
    BuildMapCloud();
  }

  const size_t num_max_points_pcd = 100000;
  size_t num_pcd_files = map_cloud_.size() / num_max_points_pcd + 1;

  if (num_pcd_files == 1) {
    const std::string fname = slam_data_dir_ + "/map_cloud.pcd";
    WriteBinaryPCD(fname, map_cloud_, map_intensities_);
    std::cout << "Wrote map cloud to " << fname << std::endl;
  } else {
    for (size_t i = 0; i < num_pcd_files; ++i) {
      PointCloud3f cloud;
      std::vector<float> intensities;
      cloud.reserve(num_max_points_pcd);
      intensities.reserve(num_max_points_pcd);
      const size_t start_idx = i * num_max_points_pcd;
      for (size_t j = start_idx; j < map_cloud_.size(); ++j) {
        const size_t idx = j - start_idx;
        if (idx == num_max_points_pcd) {
          break;
        }
        cloud.push_back(map_cloud_[j]);
        intensities.push_back(map_intensities_[j]);
      }
      const std::string fname = slam_data_dir_ + "/map_cloud" + std::to_string(i) + ".pcd";
      WriteBinaryPCD(fname, cloud, intensities);
      std::cout << "Wrote " << i + 1 << "th map cloud to " << fname << std::endl;
    }
  }

  try {
    for (const auto& entry : std::filesystem::directory_iterator(slam_data_dir_)) {
      if (entry.is_regular_file() && entry.path().extension() == ".bin") {
        std::filesystem::remove(entry.path());
        std::cout << "[INFO] Removed: " << entry.path() << std::endl;
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    std::cerr << "[ERROR] Filesystem error: " << e.what() << std::endl;
  }
}

void SLAM3DLib::GetCloudFileNames(
  size_t idx,
  std::string& raw_cloud_file,
  std::string& filtered_cloud_file) {
  const std::string id = std::to_string(idx);
  raw_cloud_file = slam_data_dir_ + "raw_cloud" + id + ".bin";
  filtered_cloud_file = slam_data_dir_ + "filtered_cloud" + id + ".bin";
}

void SLAM3DLib::BuildFilteredMapCloud() {
  const size_t num_max_points = 5000000;
  filtered_map_cloud_.clear();
  filtered_map_cloud_.reserve(num_max_points);
  for (int i = static_cast<int>(pose_graph_.size()) - 1; i >= 0; --i) {
    const Sophus::SE3f& T = pose_graph_[i];
    std::string raw_cloud_file, filtered_cloud_file;
    GetCloudFileNames(i, raw_cloud_file, filtered_cloud_file);
    PointCloud3f cloud;
    ReadPointCloud(filtered_cloud_file, cloud);
    for (const auto& p : cloud) {
      filtered_map_cloud_.push_back(T * p);
    }

    if (filtered_map_cloud_.size() > num_max_points) {
      break;
    }
  }
}

void SLAM3DLib::BuildGraphPoseKDTree() {
  graph_pose_points_.resize(pose_graph_.size());
  for (size_t i = 0; i < pose_graph_.size(); ++i) {
    graph_pose_points_[i] = pose_graph_[i].translation();
  }
  graph_pose_adaptor_ = std::make_unique<PointCloudAdaptor>(graph_pose_points_);
  graph_pose_kdtree_ = std::make_unique<KDTree>(3, *graph_pose_adaptor_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  graph_pose_kdtree_->buildIndex();
}

void SLAM3DLib::GetTargetCandidateIndices(
  const Sophus::SE3f& source_pose,
  std::vector<size_t>& target_indices) {
  if (graph_pose_points_.size() <= max_num_loop_candidates_) {
    for (size_t i = 0; i < graph_pose_points_.size(); ++i) {
      target_indices.push_back(i);
    }
  } else {
    std::vector<size_t> indices(max_num_loop_candidates_);
    std::vector<float> dists(max_num_loop_candidates_);
    nanoflann::KNNResultSet<float> result_set(max_num_loop_candidates_);
    const Eigen::Vector3f query = source_pose.translation();
    const float query_pt[3] = {query.x(), query.y(), query.z()};
    result_set.init(indices.data(), dists.data());
    graph_pose_kdtree_->findNeighbors(result_set, query_pt, nanoflann::SearchParams());
    target_indices = indices;
  }
}

void SLAM3DLib::TransformCloud(
  const Sophus::SE3f& T,
  PointCloud3f& cloud) {
  for (size_t i = 0; i < cloud.size(); ++i) {
    cloud[i] = T * cloud[i];
  }
}

void SLAM3DLib::GetGraphNodes(PointCloud3f& nodes) {
  nodes.resize(pose_graph_.size());
  for (size_t i = 0; i < pose_graph_.size(); ++i) {
    nodes[i] = pose_graph_[i].translation();
  }
}

void SLAM3DLib::GetOdomEdgePoints(
  PointCloud3f& start_points,
  PointCloud3f& end_points) {
  start_points.resize(odom_edges_.size());
  end_points.resize(odom_edges_.size());
  for (size_t i = 0; i < odom_edges_.size(); ++i) {
    const size_t src_idx = odom_edges_[i].source_idx;
    const Sophus::SE3f& src_pose = pose_graph_[src_idx];
    start_points[i] = src_pose.translation();

    const Sophus::SE3f tgt_pose = src_pose * odom_edges_[i].relative;
    end_points[i] = tgt_pose.translation();
  }
}

void SLAM3DLib::GetLoopEdgePoints(
  PointCloud3f& start_points,
  PointCloud3f& end_points) {
  start_points.resize(loop_edges_.size());
  end_points.resize(loop_edges_.size());
  for (size_t i = 0; i < loop_edges_.size(); ++i) {
    const size_t src_idx = loop_edges_[i].source_idx;
    const Sophus::SE3f& src_pose = pose_graph_[src_idx];
    start_points[i] = src_pose.translation();

    const Sophus::SE3f tgt_pose = src_pose * loop_edges_[i].relative;
    end_points[i] = tgt_pose.translation();
  }
}

} //namespace pslam
