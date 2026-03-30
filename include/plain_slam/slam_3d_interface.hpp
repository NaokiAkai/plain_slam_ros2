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
#include <cmath>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include <sophus/se3.hpp>

#include <plain_slam/types.hpp>
#include <plain_slam/io.hpp>
#include <plain_slam/keyframe_detector.hpp>
#include <plain_slam/voxel_grid_filter.hpp>
#include <plain_slam/gicp.hpp>
#include <plain_slam/graph_optimizer.hpp>

namespace pslam {

class SLAM3DInterface {
 public:
  SLAM3DInterface();

  ~SLAM3DInterface();

  void SetData(
    const Sophus::SE3f& odom_pose,
    const PointCloud3f& scan_cloud,
    const std::vector<float>& scan_intensities);

  void MakeSLAMDataDir();

  void ClearSLAMDataDir();

  void ReadSLAMParams();

  void BuildMapCloud();

  void WriteMapCloudPCD();

  void SetSLAMDataDir(const std::string& dir) {
    slam_data_dir_ = dir;
  }

  void SetParamFilesDir(const std::string& dir) {
    param_files_dir_ = dir;
  }

  void SetAccumulationCycle(size_t cycle) {
    accumulation_cycle_ = cycle;
  }

  void SetScanCloudFilterSize(float size) {
    scan_cloud_filter_size_ = size;
  }

  void SetMaxNumLoopCandidates(size_t num) {
    max_num_loop_candidates_ = num;
  }

  void SetLoopDetectionDistanceThresh(float th) {
    loop_detection_distance_thresh_ = th;
  }

  void SetLoopDetectionHeightThresh(float th) {
    loop_detection_height_thresh_ = th;
  }

  bool IsPoseGraphUpdated() const {
    return is_pose_graph_updated_;
  }

  void SetPoseGraphUpdated(bool flag) {
    is_pose_graph_updated_ = flag;
  }

  Sophus::SE3f GetSLAMPose() const {
    return slam_pose_;
  }

  const PointCloud3f& GetMapCloud() const {
    return map_cloud_;
  }

  const std::vector<Sophus::SE3f> GetPoseGraph() const {
    return pose_graph_;
  }

  const std::vector<float> GetMapIntensities() const {
    return map_intensities_;
  }

  void GetGraphNodes(PointCloud3f& nodes);

  void GetOdomEdgePoints(
    PointCloud3f& start_points,
    PointCloud3f& end_points);

  void GetLoopEdgePoints(
    PointCloud3f& start_points,
    PointCloud3f& end_points);

  const PointCloud3f& GetFilteredMapCloud() const {
    return filtered_map_cloud_;
  }

 private:
  std::string slam_data_dir_;
  std::string param_files_dir_;

  Sophus::SE3f slam_pose_;
  Sophus::SE3f prev_odom_pose_;
  Sophus::SE3f latest_keyframe_odom_pose_;
  bool is_odom_pose_initialized_;

  PointCloud3f map_cloud_;
  std::vector<float> map_intensities_;
  PointCloud3f filtered_map_cloud_;

  size_t accumulation_cycle_;
  size_t accumulated_count_;
  PointCloud3f accumulated_scan_cloud_;
  std::vector<float> accumulated_scan_intensities_;

  float scan_cloud_filter_size_;

  KeyframeDetector kf_detector_;

  GICP gicp_;

  std::vector<Sophus::SE3f> pose_graph_;
  std::vector<Edge> odom_edges_;
  std::vector<Edge> loop_edges_;

  PointCloud3f graph_pose_points_;
  std::unique_ptr<PointCloudAdaptor> graph_pose_adaptor_;
  std::unique_ptr<KDTree> graph_pose_kdtree_;

  PointCloud3f local_map_cloud_;
  std::unique_ptr<PointCloudAdaptor> local_map_adaptor_;
  std::unique_ptr<KDTree> local_map_kdtree_;
  float local_map_matching_rate_;

  size_t max_num_loop_candidates_;
  float loop_detection_distance_thresh_;
  float loop_detection_height_thresh_;
  float min_local_map_matching_rate_;
  float error_average_th_;
  float active_points_rate_th_;

  bool is_pose_graph_updated_;

  GraphOptimizer g_optimizer_;

  void GetCloudFileNames(
    size_t idx,
    std::string& raw_cloud_file,
    std::string& filtered_cloud_file);

  void BuildFilteredMapCloud();

  void BuildGraphPoseKDTree();

  void BuildLocalMapKDTree(const std::vector<size_t>& target_indices);

  void ComputeLocalMapMatchingRate(const PointCloud3f& filtered_cloud);

  void GetTargetCandidateIndices(
    const Sophus::SE3f& source_pose,
    std::vector<size_t>& target_indices);

  void TransformCloud(
    const Sophus::SE3f& T,
    PointCloud3f& cloud);
};


} // namespace pslam
