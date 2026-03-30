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

#include <plain_slam/imu_preintegrator.hpp>

namespace pslam {

IMUPreintegrator::IMUPreintegrator() {
  process_noise_cov_ = Eigen::Matrix<float, 12, 12>::Identity();
  process_noise_cov_.block<3, 3>(0, 0) *= 1e5f;
  process_noise_cov_.block<3, 3>(3, 3) *= 1e6f;
  process_noise_cov_.block<3, 3>(6, 6) *= 1e4f;
  process_noise_cov_.block<3, 3>(9, 9) *= 1e5f;
}

IMUPreintegrator::~IMUPreintegrator() {

}

void IMUPreintegrator::Preintegration(
  const std::vector<IMUMeasure>& imu_measures,
  State& state,
  StateCov& state_cov) {
  preint_Ts_.resize(imu_measures.size() + 1);
  preint_vs_.resize(imu_measures.size() + 1);

  preint_Ts_[0] = state.T;
  preint_vs_[0] = state.v;
  preint_time_ = 0.0f;

  const Eigen::Vector3f ab = state.ab;
  const Eigen::Vector3f gb = state.gb;
  const Eigen::Vector3f gacc = state.gacc;

  Eigen::Vector3f acc_sum = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro_sum = Eigen::Vector3f::Zero();

  for (size_t i = 0; i < imu_measures.size(); ++i) {
    const float dt = static_cast<float>(imu_measures[i].dt);
    const Eigen::Vector3f acc = imu_measures[i].acc - ab;
    const Eigen::Vector3f gyro = imu_measures[i].gyro - gb;
    acc_sum += acc;
    gyro_sum += gyro;

    const Eigen::Vector3f phi = gyro * dt;
    const Sophus::SO3f dR = Sophus::SO3f::exp(phi);
    const Sophus::SO3f R = preint_Ts_[i].so3() * dR;
    const Eigen::Vector3f wacc = R * acc - gacc;
    const Eigen::Vector3f v_prev = preint_vs_[i];
    const Eigen::Vector3f v = v_prev + wacc * dt;
    const Eigen::Vector3f t = preint_Ts_[i].translation()
                            + v_prev * dt + 0.5f * wacc * dt * dt;

    preint_Ts_[i + 1] = Sophus::SE3f(R, t);
    preint_vs_[i + 1] = v;
    preint_time_ += dt;
  }

  state.T = preint_Ts_.back();
  state.v = preint_vs_.back();
  delta_T_ = preint_Ts_.front().inverse() * preint_Ts_.back();
  delta_v_ = preint_vs_.back() - preint_vs_.front();

  /*
  const float dt = preint_time_;
  const float dt2 = dt * dt;
  const Eigen::Vector3f acc = acc_sum / imu_measures.size();
  const Eigen::Vector3f gyro = gyro_sum / imu_measures.size();
  const Eigen::Vector3f phi = gyro * dt;
  const Eigen::Matrix3f dR = Sophus::SO3f::exp(phi).matrix();
  const Eigen::Matrix3f AinvT = LeftJacobianInvSO3(phi).transpose();
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f R = preint_Ts_[0].rotationMatrix();
  const Eigen::Matrix3f RT = R.transpose();
  const Eigen::Matrix3f A = Sophus::SO3f::hat(acc).matrix();

  Eigen::Matrix<float, 24, 24> Fx = Eigen::Matrix<float, 24, 24>::Identity();
  Fx.block<3, 3>(0, 3) = -0.5f * R * A * RT * dt2;
  Fx.block<3, 3>(0, 6) = I * dt;
  Fx.block<3, 3>(0, 12) = -0.5f * R * dt2;
  Fx.block<3, 3>(0, 15) = 0.5f * I * dt2;

  Fx.block<3, 3>(3, 3) = dR;
  Fx.block<3, 3>(3, 9) = -R * dR * AinvT * dt;

  Fx.block<3, 3>(6, 3) = -R * A * RT * dt;
  Fx.block<3, 3>(6, 12) = -R * dt;
  Fx.block<3, 3>(6, 15) = I * dt;

  Eigen::Matrix<float, 24, 12> Fw = Eigen::Matrix<float, 24, 12>::Zero();
  Fw.block<3, 3>(0, 3) = -0.5f * R * dt2;
  Fw.block<3, 3>(3, 0) = -R * dR * AinvT * dt;
  Fw.block<3, 3>(6, 3) = -R * dt;
  Fw.block<3, 3>(9, 6) =  I * dt;
  Fw.block<3, 3>(12, 9) =  I * dt;
  */

  const float dt = preint_time_;
  const float dt2 = dt * dt;
  const Eigen::Vector3f acc = acc_sum / imu_measures.size();
  const Eigen::Vector3f gyro = gyro_sum / imu_measures.size();
  const Eigen::Vector3f phi = gyro * dt;
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f R = preint_Ts_.front().rotationMatrix();
  const Eigen::Matrix3f RT = preint_Ts_.back().rotationMatrix().transpose();
  const Eigen::Matrix3f RA = Sophus::SO3f::hat(R * acc);
  const Eigen::Matrix3f dR = Sophus::SO3f::exp(phi).matrix();
  const Eigen::Matrix3f Jlinv = LeftJacobianInvSO3(Sophus::SO3f(R * dR).log());
  const Eigen::Matrix3f Jr = RightJacobianSO3(phi);

  Eigen::Matrix<float, 24, 24> Fx = Eigen::Matrix<float, 24, 24>::Identity();
  Fx.block<3, 3>(0, 3) = -0.5f * RA * dt2;
  Fx.block<3, 3>(0, 6) = I * dt;
  Fx.block<3, 3>(0, 12) = -0.5f * R * dt2;
  Fx.block<3, 3>(0, 15) = 0.5f * I * dt2;

  Fx.block<3, 3>(3, 3) = Jlinv * RT;
  Fx.block<3, 3>(3, 9) = -Jlinv * Jr * dt;

  Fx.block<3, 3>(6, 3) = -RA * dt;
  Fx.block<3, 3>(6, 12) = -R * dt;
  Fx.block<3, 3>(6, 15) = I * dt;

  Eigen::Matrix<float, 24, 12> Fw = Eigen::Matrix<float, 24, 12>::Zero();
  Fw.block<3, 3>(0, 3) = -0.5f * R * dt2;
  Fw.block<3, 3>(3, 0) = -Jlinv * Jr * dt;
  Fw.block<3, 3>(6, 3) = -R * dt;
  Fw.block<3, 3>(9, 6) =  I * dt;
  Fw.block<3, 3>(12, 9) =  I * dt;

  state_cov = Fx * state_cov * Fx.transpose()
            + Fw * process_noise_cov_ * Fw.transpose();
}

void IMUPreintegrator::Preintegration(
  const IMUMeasure imu_measure,
  State& state,
  StateCov& state_cov) {
  const Eigen::Vector3f ab = state.ab;
  const Eigen::Vector3f gb = state.gb;
  const Eigen::Vector3f gacc = state.gacc;

  const float dt = static_cast<float>(imu_measure.dt);
  const Eigen::Vector3f acc = imu_measure.acc - ab;
  const Eigen::Vector3f gyro = imu_measure.gyro - gb;

  const Eigen::Matrix3f R_prev = state.T.rotationMatrix();
  const Eigen::Vector3f phi = gyro * dt;
  const Sophus::SO3f dR = Sophus::SO3f::exp(phi);
  const Sophus::SO3f R = state.T.so3() * dR;
  const Eigen::Vector3f wacc = R * acc - gacc;
  const Eigen::Vector3f v_prev = state.v;
  const Eigen::Vector3f v = v_prev + wacc * dt;
  const Eigen::Vector3f t = state.T.translation()
                          + v_prev * dt + 0.5f * wacc * dt * dt;

  state.T = Sophus::SE3f(R, t);
  state.v = v;
                          
  const float dt2 = dt * dt;
  const Eigen::Matrix3f AinvT = LeftJacobianInvSO3(phi).transpose();
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f RT = R_prev.transpose();
  const Eigen::Matrix3f A = Sophus::SO3f::hat(acc).matrix();

  Eigen::Matrix<float, 24, 24> Fx = Eigen::Matrix<float, 24, 24>::Identity();
  Fx.block<3, 3>(0, 3) = -0.5f * R_prev * A * RT * dt2;
  Fx.block<3, 3>(0, 6) = I * dt;
  Fx.block<3, 3>(0, 12) = -0.5f * R_prev * dt2;
  Fx.block<3, 3>(0, 15) = 0.5f * I * dt2;

  Fx.block<3, 3>(3, 3) = dR.matrix();
  Fx.block<3, 3>(3, 9) = -R_prev * dR.matrix() * AinvT * dt;

  Fx.block<3, 3>(6, 3) = -R_prev * A * RT * dt;
  Fx.block<3, 3>(6, 12) = -R_prev * dt;
  Fx.block<3, 3>(6, 15) = I * dt;

  Eigen::Matrix<float, 24, 12> Fw = Eigen::Matrix<float, 24, 12>::Zero();
  Fw.block<3, 3>(0, 3) = -0.5f * R_prev * dt2;
  Fw.block<3, 3>(3, 0) = -R_prev * A * RT * dt;
  Fw.block<3, 3>(6, 3) = -R_prev * dt;
  Fw.block<3, 3>(9, 6) =  I * dt;
  Fw.block<3, 3>(12, 9) =  I * dt;

  state_cov = Fx * state_cov * Fx.transpose()
            + Fw * process_noise_cov_ * Fw.transpose();
}

void IMUPreintegrator::DeskewScanCloud(
  const State& state,
  const std::vector<IMUMeasure> imu_measures,
  const std::vector<double>& scan_stamps,
  PointCloud3f& scan_cloud) {
  const Eigen::Vector3f ab = state.ab;
  const Eigen::Vector3f gb = state.gb;
  const Eigen::Vector3f gacc = state.gacc;
  const Sophus::SE3f Tio = preint_Ts_.back().inverse();

  for (size_t i = 0; i < scan_cloud.size(); ++i) {
    const double stamp = scan_stamps[i];
    int idx = -1;
    for (size_t j = 1; j < imu_measures.size(); ++j) {
      if (stamp < imu_measures[j].stamp) {
        idx = j - 1;
        break;
      }
    }

    if (idx < 0) {
      continue;
    }

    const float dt = static_cast<float>(stamp - imu_measures[idx].stamp);
    const Eigen::Vector3f acc = imu_measures[idx].acc - ab;
    const Eigen::Vector3f gyro = imu_measures[idx].gyro - gb;

    const Sophus::SO3f dR = Sophus::SO3f::exp(gyro * dt);
    const Sophus::SO3f R = preint_Ts_[idx].so3() * dR;
    const Eigen::Vector3f wacc = R * acc - gacc;
    const Eigen::Vector3f v_prev = preint_vs_[idx];
    const Eigen::Vector3f t = preint_Ts_[idx].translation()
                            + v_prev * dt + 0.5f * wacc * dt * dt;
    const Sophus::SE3f Toi(R, t);

    scan_cloud[i] = Tio * Toi * scan_cloud[i];
  }
}

} // namespace pslam
