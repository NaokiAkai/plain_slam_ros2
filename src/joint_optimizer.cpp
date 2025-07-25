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

#include <plain_slam/joint_optimizer.hpp>

namespace pslam {

JointOptimizer::JointOptimizer() {
  huber_delta_ = 0.5f;
}

JointOptimizer::~JointOptimizer() {

}

bool JointOptimizer::Estimate(
  const State& pred_state,
  const StateCov& pred_state_cov,
  const PointCloud3f& scan_cloud,
  size_t max_iter_num,
  size_t num_max_matching_points,
  float max_correspondence_dist,
  float convergence_th,
  State& updated_state,
  StateCov& updated_cov,
  NormalMap& normal_map) {
  const State initial = pred_state;
  const Eigen::Matrix3f initial_R = initial.T.rotationMatrix();
  const Eigen::Matrix3f initial_Ril = initial.Til.rotationMatrix();

  int scan_step = scan_cloud.size() / num_max_matching_points;
  if (scan_step < 1) {
    scan_step = 1;
  }

  State state = pred_state;
  StateCov cov = pred_state_cov;
  bool has_converged = false;

  using Matrix1x6f = Eigen::Matrix<float, 1, 6>;

  for (size_t iter_num = 0; iter_num < max_iter_num; ++iter_num) {
    size_t used_points_num = 0;
    size_t corresp_num = 0;

    std::vector<float> rs;
    std::vector<Matrix1x6f, Eigen::aligned_allocator<Matrix1x6f>> Jois;
    std::vector<Matrix1x6f, Eigen::aligned_allocator<Matrix1x6f>> Jils;
    rs.reserve(num_max_matching_points);
    Jois.reserve(num_max_matching_points);
    Jils.reserve(num_max_matching_points);

    const Sophus::SE3f Tli = state.Til.inverse();
    const Eigen::Matrix3f Roi = state.T.rotationMatrix();
    const Eigen::Matrix3f Ril = state.Til.rotationMatrix();

    for (size_t i = 0; i < scan_cloud.size(); i += scan_step) {
      used_points_num++;

      const Eigen::Vector3f query = state.T * scan_cloud[i];
      Eigen::Vector3f target, normal;
      if (!normal_map.FindCorrespondence(query, max_correspondence_dist, target, normal)) {
        continue;
      }

      const float r = (target - query).transpose() * normal;
      rs.push_back(r);

      const Eigen::Vector3f dr_dtoi = -normal;
      const Eigen::Vector3f Rpi = state.T.rotationMatrix() * scan_cloud[i];
      const Eigen::Matrix<float, 1, 3> nT = normal.transpose();
      const Eigen::Vector3f dz_dRoi = nT * Sophus::SO3f::hat(Rpi).matrix();
      Eigen::Matrix<float, 1, 6> Joi;
      Joi << dr_dtoi(0), dr_dtoi(1), dr_dtoi(2), dz_dRoi(0), dz_dRoi(1), dz_dRoi(2);
      Jois.emplace_back(Joi);

      const Eigen::Vector3f dr_dtil = nT * Roi;
      const Eigen::Vector3f pl = Tli * scan_cloud[i];
      const Eigen::Vector3f dz_dRil = nT * Roi * Sophus::SO3f::hat(Ril * pl).matrix();
      Eigen::Matrix<float, 1, 6> Jil;
      Jil << dr_dtil(0), dr_dtil(1), dr_dtil(2), dz_dRil(0), dz_dRil(1), dz_dRil(2);
      Jils.emplace_back(Jil);

      corresp_num++;
      if (corresp_num >= num_max_matching_points) {
        break;
      }
    }

    active_points_rate_ = static_cast<float>(corresp_num)
                        / static_cast<float>(used_points_num);

    const float rinv = 1e-6f;
    Eigen::Matrix<float, Eigen::Dynamic, 1> r(corresp_num, 1);
    Eigen::Matrix<float, Eigen::Dynamic, 24> J
      = Eigen::Matrix<float, Eigen::Dynamic, 24>::Zero(corresp_num, 24);
    Eigen::Matrix<float, 24, Eigen::Dynamic> JT
      = Eigen::Matrix<float, 24, Eigen::Dynamic>::Zero(24, corresp_num);
    // Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Rinv = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(corresp_num, corresp_num);;
    for (size_t i = 0; i < corresp_num; ++i) {
      if (huber_delta_ > 0.0f) {
        const float abs_r = std::abs(rs[i]);
        const float w = (abs_r <= huber_delta_) ? 1.0f : (huber_delta_ / abs_r);
        r(i, 0) = w * rs[i];
        J.block<1, 6>(i, 0) = w * Jois[i];
        JT.block<6, 1>(0, i) = w * Jois[i].transpose();
        J.block<1, 6>(i, 18) = w * Jils[i];
        JT.block<6, 1>(18, i) = w * Jils[i].transpose();
      } else {
        r(i, 0) = rs[i];
        J.block<1, 6>(i, 0) = Jois[i];
        JT.block<6, 1>(0, i) = Jois[i].transpose();
        J.block<1, 6>(i, 18) = Jils[i];
        JT.block<6, 1>(18, i) = Jils[i].transpose();
      }
      // Rinv(i, i) = rinv;
    }

    const Eigen::Matrix<float, 24, 24> I = Eigen::Matrix<float, 24, 24>::Identity();

    Eigen::Matrix<float, 24, 24> H = I;
    const Eigen::Matrix3f dR = initial_R.transpose() * state.T.rotationMatrix();  
    const Eigen::Matrix3f dRil = initial_Ril.transpose() * state.Til.rotationMatrix();  
    const Eigen::Vector3f dtheta = Sophus::SO3f(dR).log();
    const Eigen::Vector3f dthetail = Sophus::SO3f(dRil).log();
    H.block<3, 3>(3, 3) = LeftJacobianInvSO3(dtheta).transpose();
    H.block<3, 3>(21, 21) = LeftJacobianInvSO3(dthetail).transpose();
    const Eigen::Matrix<float, 24, 24> Hinv = H.ldlt().solve(I);

    const Eigen::Matrix<float, 24, 24> P = Hinv * cov * Hinv.transpose();
    const Eigen::Matrix<float, 24, 24> Pinv = P.ldlt().solve(I);

    // const Eigen::Matrix<float, 24, Eigen::Dynamic> JT = J.transpose();
    // const Eigen::Matrix<float, 24, 24> JTRinvJPinv = JT * Rinv * J + Pinv;
    const Eigen::Matrix<float, 24, 24> JTRinvJPinv = rinv * JT * J + Pinv;
    const Eigen::Matrix<float, 24, 24> denom = JTRinvJPinv.ldlt().solve(I);
    // const Eigen::Matrix<float, 24, Eigen::Dynamic> K = denom * JT * Rinv;
    const Eigen::Matrix<float, 24, Eigen::Dynamic> K = rinv * denom * JT;

    Eigen::Matrix<float, 24, 1> dx;
    dx.block<3, 1>(0, 0) = state.T.translation() - initial.T.translation();
    dx.block<3, 1>(3, 0) = (initial.T.so3().inverse() * state.T.so3()).log();
    dx.block<3, 1>(6, 0) = state.v - initial.v;
    dx.block<3, 1>(9, 0) = state.gb - initial.gb;
    dx.block<3, 1>(12, 0) = state.ab - initial.ab;
    dx.block<3, 1>(15, 0) = state.gacc - initial.gacc;
    dx.block<3, 1>(18, 0) = state.Til.translation() - initial.Til.translation();
    dx.block<3, 1>(21, 0) = (initial.Til.so3().inverse() * state.Til.so3()).log();

    const Eigen::Matrix<float, 24, 1> d = -K * r - (I - K * J) * Hinv * dx;
    const Eigen::Vector3f tn = state.T.translation() + d.block<3, 1>(0, 0);
    const Sophus::SO3f Rn = Sophus::SO3f::exp(d.block<3, 1>(3, 0)) * state.T.so3();
    state.T = Sophus::SE3f(Rn, tn);
    state.v += d.block<3, 1>(6, 0);
    state.gb += d.block<3, 1>(9, 0);
    state.ab += d.block<3, 1>(12, 0);
    state.gacc += d.block<3, 1>(15, 0);
    const Eigen::Vector3f tiln = state.Til.translation() + d.block<3, 1>(18, 0);
    const Sophus::SO3f Riln = Sophus::SO3f::exp(d.block<3, 1>(21, 0)) * state.Til.so3();
    state.Til = Sophus::SE3f(Riln, tiln);

    cov = (I - K * J) * P;

    const float epsilon = d.norm();
    // std::cout << "iteration = " << iter_num + 1 << ", epsilon = " << epsilon << std::endl;
    if (epsilon < convergence_th) {
      has_converged = true;
      break;
    }
  }

  if (!has_converged) {
    return false;
  }

  updated_state = state;
  updated_cov = cov;

  return true;
}

} // namespace pslam
