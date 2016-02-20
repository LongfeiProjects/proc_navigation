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
#include <lib_atlas/maths/stats.h>
#include "proc_navigation/controllers/baro_controller.h"
#include "proc_navigation/controllers/dvl_controller.h"
#include "proc_navigation/controllers/imu_controller.h"
#include "proc_navigation/kalman/extended_kalman_filter.h"

namespace proc_navigation {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ExtendedKalmanFilter::ExtendedKalmanFilter(const EkfConfiguration &conf)
    ATLAS_NOEXCEPT : EkfConfiguration(conf),
                     new_data_ready_(false),
                     init_timer_(),
                     processing_mutex_() {}

//------------------------------------------------------------------------------
//
ExtendedKalmanFilter::~ExtendedKalmanFilter() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::OnSubjectNotify(atlas::Subject<> &subject)
    ATLAS_NOEXCEPT {
  if (dynamic_cast<BaroController *>(&subject) != nullptr) {
    UpdateBaroData();
  } else if (dynamic_cast<DvlController *>(&subject) != nullptr) {
    UpdateDvlData();
  } else if (dynamic_cast<ImuController *>(&subject) != nullptr) {
    UpdateImuData();
  }
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::Initiate() {
  init_timer_.Start();

  // The vectors of interest for the initilization states.
  std::array<std::vector<double>, 3> g;
  std::array<std::vector<double>, 3> m;

  while(init_timer_.MicroSeconds() < t_init) {
    if()
  }

  CalculateImuMeans(g);
  CalculateMagMeans(m);

  // The initialization is finished, start the Kalman filter here
  Start();
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::CalculateImuMeans(const std::array<std::vector<double>,
                                                              3> &g) ATLAS_NOEXCEPT {
  // WARN: Attention should be paid to the values of this vector.
  // If the IMU is inverted, there will be a -1 factor.
  Eigen::Vector3d g_mean;
  g_mean(0) = imu_sign_x * atlas::Mean(std::get<0>(g));
  g_mean(1) = imu_sign_y * atlas::Mean(std::get<1>(g));
  g_mean(2) = imu_sign_z * atlas::Mean(std::get<2>(g));
  init_state_.ge = g_mean.norm();

  // Calculate initial roll angle - Equation 10.14 - Farrell
  init_state_.roll = std::atan2(g_mean(1), g_mean(2));

  // Calculate initial roll angle - Equation 10.15 - Farrell
  init_state_.pitch = std::atan2(-g_mean(0), std::sqrt(g_mean(1)*g_mean(1) + g_mean(2)*g_mean(2)));

  // Calculate rotation matrix - Equation 10.16 - Farrell
  // The row index is passed first on a Eigen::Matrix, for more info, see:
  // http://eigen.tuxfamily.org/dox-devel/group__TutorialMatrixClass.html#title4
  init_state_.r_b2w(0, 0) = std::cos(init_state_.pitch);
  init_state_.r_b2w(0, 1) = std::sin(init_state_.pitch)*std::sin(init_state_.roll);
  init_state_.r_b2w(0, 2) = std::sin(init_state_.pitch)*std::cos(init_state_.roll);
  init_state_.r_b2w(1, 0) = 0;
  init_state_.r_b2w(1, 1) = std::cos(init_state_.roll);
  init_state_.r_b2w(1, 2) = -std::sin(init_state_.roll);
  init_state_.r_b2w(2, 0) = -std::sin(init_state_.pitch);
  init_state_.r_b2w(2, 1) = std::cos(init_state_.pitch)*std::sin(init_state_.roll);
  init_state_.r_b2w(2, 2) = std::cos(init_state_.pitch)*std::cos(init_state_.roll);
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::CalculateMagMeans(const std::array<std::vector<double>,
                                                              3> &m) ATLAS_NOEXCEPT {
  Eigen::Vector3d m_mean;
  m_mean(0) = atlas::Mean(std::get<0>(m));
  m_mean(1) = atlas::Mean(std::get<1>(m));
  m_mean(2) = atlas::Mean(std::get<2>(m));
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::Run() {
  while (IsRunning()) {
    if (new_data_ready_) {
      std::lock_guard<std::mutex> guard(processing_mutex_);
    }
  }
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateImuData() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(processing_mutex_);
  // TODO Thibaut Mattio: Get the IMU data here
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateDvlData() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(processing_mutex_);
  // TODO Thibaut Mattio: Get the DVL data here
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateBaroData() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(processing_mutex_);
  // TODO Thibaut Mattio: Get the Baro data here
}

//------------------------------------------------------------------------------
//
bool ExtendedKalmanFilter::IsNewDataReady() const ATLAS_NOEXCEPT {
  return new_data_ready_;
}

}  // namespace proc_navigation
