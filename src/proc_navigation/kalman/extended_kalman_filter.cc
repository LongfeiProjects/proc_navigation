/**
 * \file	object_mapper.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	07/02/2016
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <eigen3/Eigen/Eigen>
#include <lib_atlas/maths.h>
#include "proc_navigation/kalman/extended_kalman_filter.h"

namespace proc_navigation {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ExtendedKalmanFilter::ExtendedKalmanFilter(
    const StateController<BaroMessage>::Ptr &baro,
    const StateController<ImuMessage>::Ptr &imu,
    const StateController<MagMessage>::Ptr &mag,
    const StateController<DvlMessage>::Ptr &dvl,
    const EkfConfiguration &conf) ATLAS_NOEXCEPT : atlas::Observer<>(),
                                                   atlas::Runnable(),
                                                   EkfConfiguration(conf),
                                                   baro_(baro),
                                                   imu_(imu),
                                                   mag_(mag),
                                                   dvl_(dvl),
                                                   processing_mutex_(),
                                                   init_timer_(),
                                                   x0_(),
                                                   qc_(),
                                                   p0_(),
                                                   x_(),
                                                   ge_() {
  // Initialize the Kalman filter here
  Initialize();

  // The initialization is finished, start the Kalman filter here
  Start();
}

//------------------------------------------------------------------------------
//
ExtendedKalmanFilter::~ExtendedKalmanFilter() ATLAS_NOEXCEPT { }

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::OnSubjectNotify(atlas::Subject<> &subject)
ATLAS_NOEXCEPT {
  if (dynamic_cast<StateController<BaroMessage> *>(&subject) != nullptr &&
      active_baro) {
    UpdateBaroData();
  } else if (dynamic_cast<StateController<DvlMessage> *>(&subject) != nullptr &&
      active_dvl) {
    UpdateDvlData();
  } else if (dynamic_cast<StateController<MagMessage> *>(&subject) != nullptr &&
      active_mag) {
    UpdateMagData();
  } else if (dynamic_cast<StateController<ImuMessage> *>(&subject) != nullptr) {
    UpdateImuData();
  }
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::Initialize() {
  init_timer_.Start();

  // The vectors of interest for the initilization states.
  std::array<std::vector<double>, 3> g;
  std::array<std::vector<double>, 3> m;

  while (init_timer_.MicroSeconds() < t_init) {
    // Check if there is new data here and add to the vectors
  }

  x0_.pos = Eigen::Vector3d(0, 0, 0);
  x0_.vel = Eigen::Vector3d(0, 0, 0);
  x0_.b = CalculateInitialRotationMatrix(g, m);
  x0_.acc_bias = Eigen::Vector3d(0, 0, 0);
  x0_.gyro_bias = Eigen::Vector3d(0, 0, 0);
  x0_.baro_bias = 0;

  qc_(0, 0) = sigma_meas_acc;
  qc_(1, 1) = sigma_meas_acc;
  qc_(2, 2) = sigma_meas_acc;
  qc_(3, 3) = sigma_meas_gyr;
  qc_(4, 4) = sigma_meas_gyr;
  qc_(5, 5) = sigma_meas_gyr;
  qc_(6, 6) = sigma_walk_bias_acc;
  qc_(7, 7) = sigma_walk_bias_acc;
  qc_(8, 8) = sigma_walk_bias_acc;
  qc_(9, 9) = sigma_walk_bias_gyr;
  qc_(10, 10) = sigma_walk_bias_gyr;
  qc_(11, 11) = sigma_walk_bias_gyr;
  qc_(12, 12) = sigma_walk_bias_baro;

  p0_(0, 0) = sigma0_pos_x;
  p0_(1, 1) = sigma0_pos_y;
  p0_(2, 2) = sigma0_pos_z;
  p0_(3, 3) = sigma0_vel_x;
  p0_(4, 4) = sigma0_vel_y;
  p0_(5, 5) = sigma0_vel_z;
  p0_(6, 6) = sigma0_rho_x;
  p0_(7, 7) = sigma0_rho_y;
  p0_(8, 8) = sigma0_rho_z;
  p0_(9, 9) = sigma0_bias_acc;
  p0_(10, 10) = sigma0_bias_acc;
  p0_(11, 11) = sigma0_bias_acc;
  p0_(12, 12) = sigma0_bias_gyr;
  p0_(13, 13) = sigma0_bias_gyr;
  p0_(14, 14) = sigma0_bias_gyr;
  p0_(15, 15) = sigma0_bias_baro;
}

//------------------------------------------------------------------------------
//
Eigen::Quaterniond ExtendedKalmanFilter::CalculateInitialRotationMatrix(
    const std::array<std::vector<double>, 3> &g,
    const std::array<std::vector<double>, 3> &m) ATLAS_NOEXCEPT {
  // WARN: Attention should be paid to the values of this vector.
  // If the IMU is inverted, there will be a -1 factor.
  Eigen::Vector3d g_mean;
  g_mean(0) = imu_sign_x * atlas::Mean(std::get<0>(g));
  g_mean(1) = imu_sign_y * atlas::Mean(std::get<1>(g));
  g_mean(2) = imu_sign_z * atlas::Mean(std::get<2>(g));
  double ge_ = g_mean.norm();

  // Calculate initial roll angle - Equation 10.14 - Farrell
  double roll = std::atan2(g_mean(1), g_mean(2));

  // Calculate initial roll angle - Equation 10.15 - Farrell
  double pitch = std::atan2(
      -g_mean(0), std::sqrt(g_mean(1) * g_mean(1) + g_mean(2) * g_mean(2)));

  // Calculate rotation matrix - Equation 10.16 - Farrell
  Eigen::Matrix3d r_b2w;
  // The row index is passed first on a Eigen::Matrix, for more info, see:
  // http://eigen.tuxfamily.org/dox-devel/group__TutorialMatrixClass.html#title4
  r_b2w(0, 0) = std::cos(pitch);
  r_b2w(0, 1) = std::sin(pitch) * std::sin(roll);
  r_b2w(0, 2) = std::sin(pitch) * std::cos(roll);
  r_b2w(1, 0) = 0;
  r_b2w(1, 1) = std::cos(roll);
  r_b2w(1, 2) = -std::sin(roll);
  r_b2w(2, 0) = -std::sin(pitch);
  r_b2w(2, 1) = std::cos(pitch) * std::sin(roll);
  r_b2w(2, 2) = std::cos(pitch) * std::cos(roll);

  Eigen::Vector3d m_b;
  m_b(0) = mag_sign_x * atlas::Mean(std::get<0>(m));
  m_b(1) = mag_sign_x * atlas::Mean(std::get<1>(m));
  m_b(2) = mag_sign_x * atlas::Mean(std::get<2>(m));

  Eigen::Vector3d m_w = r_b2w * m_b;

  double yaw = std::atan2(-m_w(1), m_w(0));

  Eigen::Matrix3d r0_bn;
  r0_bn(0, 0) = std::cos(pitch) * std::cos(yaw);
  r0_bn(0, 1) = std::sin(roll) * std::sin(pitch) * std::cos(yaw) -
      std::cos(roll) * std::sin(yaw);
  r0_bn(0, 2) = std::cos(roll) * std::sin(pitch) * std::cos(yaw) +
      std::sin(roll) * std::sin(yaw);
  r0_bn(1, 0) = std::cos(pitch) * std::sin(yaw);
  r0_bn(1, 1) = std::sin(roll) * std::sin(pitch) * std::sin(yaw) +
      std::cos(roll) * std::cos(yaw);
  r0_bn(1, 2) = std::cos(roll) * std::sin(pitch) * std::sin(yaw) -
      std::sin(roll) * std::cos(yaw);
  r0_bn(2, 0) = -std::sin(pitch);
  r0_bn(2, 1) = std::sin(roll) * std::cos(pitch);
  r0_bn(2, 2) = std::cos(roll) * std::cos(pitch);

  Eigen::Matrix3d r0_nb = r0_bn.transpose();

  // The eigen quaternion is taking a rotation matrix as constructor parameter
  // for its initilization.
  return Eigen::Quaterniond(r0_nb);
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::Run() {
  while (IsRunning()) {
    if (IsNewDataReady()) {
      std::lock_guard<std::mutex> guard(processing_mutex_);
    }
  }
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateImuData() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(processing_mutex_);
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateDvlData() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(processing_mutex_);
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateBaroData() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(processing_mutex_);
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateMagData() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(processing_mutex_);
}

//------------------------------------------------------------------------------
//
bool ExtendedKalmanFilter::IsNewDataReady() const ATLAS_NOEXCEPT {
  return baro_->IsNewDataReady() || imu_->IsNewDataReady() ||
      dvl_->IsNewDataReady() || mag_->IsNewDataReady();
}

}  // namespace proc_navigation
