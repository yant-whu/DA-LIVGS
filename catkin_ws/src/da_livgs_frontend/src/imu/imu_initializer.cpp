/*
 * DA-LIVGS: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline
 * Copyright (C) 2023 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <glog/logging.h>
#include <imu/imu_initializer.h>
#include <algorithm>
#include <cmath>
#include <iomanip>

namespace {

double Median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2 != 0) return upper;
  std::nth_element(values.begin(), values.begin() + middle - 1,
                   values.begin() + middle);
  return 0.5 * (values[middle - 1] + upper);
}

Eigen::Matrix3d GravityAlignedRotation(const Eigen::Vector3d &acceleration) {
  const Eigen::Vector3d z_axis = acceleration.normalized();
  Eigen::Index reference_index = 0;
  z_axis.cwiseAbs().minCoeff(&reference_index);
  const Eigen::Vector3d reference =
      Eigen::Vector3d::Unit(static_cast<int>(reference_index));
  const Eigen::Vector3d x_axis =
      (reference - z_axis * z_axis.dot(reference)).normalized();
  const Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
  Eigen::Matrix3d rotation_global_to_initial;
  rotation_global_to_initial.col(0) = x_axis;
  rotation_global_to_initial.col(1) = y_axis;
  rotation_global_to_initial.col(2) = z_axis;
  return rotation_global_to_initial;
}

}  // namespace

namespace da_livgs_frontend {

IMUInitializer::IMUInitializer(const YAML::Node &node)
    : moving_initialization_enabled_(false),
      initial_done_(false),
      initialization_mode_(IMUInitializationMode::None) {
  window_length_ = yaml::GetValue<double>(node, "window_length", 1.0) * 1e9;
  imu_excite_threshold_ =
      yaml::GetValue<double>(node, "imu_excite_threshold", 1.0);

  double gravity_mag = yaml::GetValue<double>(node, "gravity_mag", 9.80);
  gravity_ << 0, 0, gravity_mag;
  moving_initialization_enabled_ = yaml::GetValue<bool>(
      node, "moving_initialization_enabled", false);
  moving_initialization_window_length_ = yaml::GetValue<double>(
      node, "moving_initialization_window_seconds", 2.0) * 1e9;
  moving_minimum_acceleration_norm_ = yaml::GetValue<double>(
      node, "moving_initialization_minimum_acceleration_norm", 0.75 * gravity_mag);
  moving_maximum_acceleration_norm_ = yaml::GetValue<double>(
      node, "moving_initialization_maximum_acceleration_norm", 1.25 * gravity_mag);
  moving_maximum_acceleration_residual_mad_ = yaml::GetValue<double>(
      node, "moving_initialization_maximum_acceleration_residual_mad", 3.0);
  moving_maximum_gyroscope_norm_median_ = yaml::GetValue<double>(
      node, "moving_initialization_maximum_gyroscope_norm_median", 1.0);
}

void IMUInitializer::FeedIMUData(const IMUData &imu_data) {
  imu_datas_.push_back(imu_data);

  // auto it0 = imu_datas_.begin();
  // while (it0 != imu_datas_.end() &&
  //        it0->timestamp < imu_data.timestamp - 2 * window_length_) {
  //   it0 = imu_datas_.erase(it0);
  // }
}

bool IMUInitializer::InitialIMUState() {
  if (StaticInitialIMUState()) return true;

  if (!moving_initialization_enabled_) return false;

  if (ActiveInitialIMUState()) return true;

  return MovingInitialIMUState();
}

bool IMUInitializer::ActiveInitialIMUState() {
  if (imu_datas_.empty()) return false;

  Eigen::Quaterniond q_last = imu_datas_.back().orientation.unit_quaternion();
  if (std::fabs(q_last.x()) > 1e-9) {
    Eigen::Matrix3d R_I0toG = q_last.toRotationMatrix();
    double yaw = R2ypr(R_I0toG).x();
    R_I0toG = (ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R_I0toG).eval();

    Eigen::Vector3d g_inI0 = R_I0toG * gravity_;

    imu_state_.q = R_I0toG;
    imu_state_.bias.accel_bias = Eigen::Vector3d::Zero();
    imu_state_.bias.gyro_bias = Eigen::Vector3d::Zero();
    imu_state_.g = gravity_;  // gw

    // LOG(INFO) << "[InertialInitializer] ActiveInitialIMUState"
    //           << "first imu time: " << imu_datas_.front().timestamp
    //           << "\nimu_datas_: " << imu_datas_.size()
    //           << "\nAccel bias : " << imu_state_.bias.accel_bias.transpose()
    //           << "\nGyro bias : " << imu_state_.bias.gyro_bias.transpose()
    //           << "\ngravity in I0: " << g_inI0.transpose();

    initial_done_ = true;
    initialization_mode_ = IMUInitializationMode::Orientation;
    return true;
  }

  return false;
}

bool IMUInitializer::StaticInitialIMUState() {
  if (imu_datas_.empty()) return false;

  if (imu_datas_.back().timestamp - imu_datas_.begin()->timestamp <
      window_length_)
    return false;

  Eigen::Vector3d accel_avg(0, 0, 0);
  Eigen::Vector3d gyro_avg(0, 0, 0);

  std::vector<IMUData> imu_cache;
  for (size_t i = imu_datas_.size() - 1; i >= 0; i--) {
    accel_avg += imu_datas_[i].accel;
    gyro_avg += imu_datas_[i].gyro;
    imu_cache.push_back(imu_datas_[i]);
    if (imu_datas_.back().timestamp - imu_datas_[i].timestamp >= window_length_)
      break;
  }
  accel_avg /= (int)imu_cache.size();
  gyro_avg /= (int)imu_cache.size();

  double accel_var = 0;
  for (size_t i = 0; i < imu_cache.size(); i++) {
    accel_var +=
        (imu_cache[i].accel - accel_avg).dot(imu_cache[i].accel - accel_avg);
  }
  accel_var = std::sqrt(accel_var / ((int)imu_cache.size() - 1));

  if (accel_var >= imu_excite_threshold_) {
    // LOG(WARNING) << "[IMUInitializer] Dont Move !";
    return false;
  }

  Eigen::Matrix3d Rot = GravityAlignedRotation(accel_avg);  // R_GtoI0
  Eigen::Vector3d g_inI0 = Rot * gravity_;

  Eigen::Matrix3d R_I0toG = Rot.inverse();
  // double yaw = R2ypr(R_I0toG).x();
  // LOG(INFO) << "[yaw] " << yaw;
  // R_I0toG = (ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R_I0toG).eval();
  imu_state_.q = R_I0toG;
  imu_state_.bias.accel_bias = accel_avg - g_inI0;
  imu_state_.bias.gyro_bias = gyro_avg;
  imu_state_.g = gravity_;  // gw

  // LOG(INFO) << "[InertialInitializer] " << "first imu time : " << imu_datas_.front().timestamp * 1e-9;
  // LOG(INFO) << "[InertialInitializer] " << "imu_datas_ : " << imu_datas_.size();
  // LOG(INFO) << "[InertialInitializer] " << "imu_cache : " << imu_datas_.size();
  // LOG(INFO) << "[InertialInitializer] " << "Accel bias : " << imu_state_.bias.accel_bias.transpose();
  // LOG(INFO) << "[InertialInitializer] " << "Gyro bias : " << imu_state_.bias.gyro_bias.transpose();
  // LOG(INFO) << "[InertialInitializer] " << "gravity in I0 : " << g_inI0.transpose();
  // std::cout << MAGENTA << "[Rwb0]\n" << R_I0toG << RESET << std::endl;

  initial_done_ = true;
  initialization_mode_ = IMUInitializationMode::Static;
  return true;
}

bool IMUInitializer::MovingInitialIMUState() {
  if (!moving_initialization_enabled_ || imu_datas_.empty()) return false;
  if (imu_datas_.back().timestamp - imu_datas_.front().timestamp <
      moving_initialization_window_length_) {
    return false;
  }

  const int64_t window_start =
      imu_datas_.front().timestamp + moving_initialization_window_length_;
  std::vector<double> acceleration[3];
  std::vector<double> gyroscope_norms;
  for (const auto &sample : imu_datas_) {
    if (sample.timestamp > window_start) break;
    for (int axis = 0; axis < 3; ++axis)
      acceleration[axis].push_back(sample.accel[axis]);
    gyroscope_norms.push_back(sample.gyro.norm());
  }
  if (gyroscope_norms.size() < 2) return false;

  Eigen::Vector3d robust_acceleration(
      Median(acceleration[0]), Median(acceleration[1]), Median(acceleration[2]));
  const double acceleration_norm = robust_acceleration.norm();
  std::vector<double> acceleration_residuals;
  acceleration_residuals.reserve(gyroscope_norms.size());
  for (size_t index = 0; index < gyroscope_norms.size(); ++index) {
    const Eigen::Vector3d sample(acceleration[0][index], acceleration[1][index],
                                 acceleration[2][index]);
    acceleration_residuals.push_back((sample - robust_acceleration).norm());
  }
  const double residual_mad = Median(acceleration_residuals);
  const double gyroscope_median = Median(gyroscope_norms);
  if (!std::isfinite(acceleration_norm) || !std::isfinite(residual_mad) ||
      !std::isfinite(gyroscope_median) ||
      acceleration_norm < moving_minimum_acceleration_norm_ ||
      acceleration_norm > moving_maximum_acceleration_norm_ ||
      residual_mad > moving_maximum_acceleration_residual_mad_ ||
      gyroscope_median > moving_maximum_gyroscope_norm_median_) {
    return false;
  }

  const Eigen::Matrix3d rotation_global_to_initial =
      GravityAlignedRotation(robust_acceleration);
  imu_state_.q = rotation_global_to_initial.inverse();
  imu_state_.bias.accel_bias.setZero();
  imu_state_.bias.gyro_bias.setZero();
  imu_state_.g = gravity_;
  initial_done_ = true;
  initialization_mode_ = IMUInitializationMode::MovingStart;
  return true;
}

}  // namespace da_livgs_frontend
