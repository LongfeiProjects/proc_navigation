/**
 * \file	extended_kalman_filter.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Adrien Kerroux <adrienkerroux@gmail.com>
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

#include "proc_navigation/kalman/extended_kalman_filter.h"
#include <lib_atlas/maths.h>
#include <eigen3/Eigen/Eigen>

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
    const EkfConfiguration &conf) ATLAS_NOEXCEPT : atlas::Runnable(),
                                                   EkfConfiguration(conf),
                                                   atlas::Subject<>(),
                                                   baro_(baro),
                                                   imu_(imu),
                                                   mag_(mag),
                                                   dvl_(dvl),
                                                   baro_timer_(),
                                                   imu_timer_(),
                                                   mag_timer_(),
                                                   dvl_timer_(),
                                                   init_timer_(),
                                                   states_(),
                                                   kalman_states_(),
                                                   extra_states_(),
                                                   kalman_matrix_(),
                                                   criterions_(),
                                                   is_stationnary_(false),
                                                   ge_(),
                                                   g_n_(),
                                                   i_(0) {
  Start();
}

//------------------------------------------------------------------------------
//
ExtendedKalmanFilter::~ExtendedKalmanFilter() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::Initialize() {
  while (!imu_->IsNewDataReady())
    ;

  // The vectors of interest for the initilization states.
  std::array<std::vector<double>, 3> g;
  std::array<std::vector<double>, 3> m;
  std::array<std::vector<double>, 3> vel;
  std::vector<double> pressure;

  init_timer_.Start();
  double real_stamp_imu = 0;
  double real_stamp_dvl = 0;
  double real_stamp_baro = 0;
  double real_stamp_mag = 0;

  while ((init_timer_.MicroSeconds() * std::pow(10, -6) < t_init) &&
         (real_stamp_imu < t_init) && (real_stamp_dvl < t_init) &&
         (real_stamp_baro < t_init) && (real_stamp_mag < t_init)) {
    if (imu_->IsNewDataReady()) {
      real_stamp_mag += imu_->GetDeltaTime();
      if (real_stamp_imu < t_init) {
        auto imu_msg = imu_->GetLastData();
        CorrectNaN(imu_msg);
        std::get<0>(g).push_back(imu_msg.linear_acceleration.x);
        std::get<1>(g).push_back(imu_msg.linear_acceleration.y);
        std::get<2>(g).push_back(imu_msg.linear_acceleration.z);
      }
    }

    if (mag_->IsNewDataReady() && active_mag) {
      real_stamp_mag += mag_->GetDeltaTime();
      if (real_stamp_mag < t_init) {
        auto mag_msg = mag_->GetLastData();
        CorrectNaN(mag_msg);
        std::get<0>(m).push_back(mag_msg.magnetic_field.x);
        std::get<1>(m).push_back(mag_msg.magnetic_field.y);
        std::get<2>(m).push_back(mag_msg.magnetic_field.z);
      }
    }

    if (dvl_->IsNewDataReady() && active_dvl) {
      real_stamp_dvl += dvl_->GetDeltaTime();
      if (real_stamp_dvl < t_init) {
        auto dvl_msg = dvl_->GetLastData();
        CorrectNaN(dvl_msg);
        std::get<0>(vel).push_back(dvl_msg.twist.twist.linear.x);
        std::get<1>(vel).push_back(dvl_msg.twist.twist.linear.y);
        std::get<2>(vel).push_back(dvl_msg.twist.twist.linear.z);
      }
    }

    if (baro_->IsNewDataReady() && active_baro) {
      real_stamp_baro += baro_->GetDeltaTime();
      if (real_stamp_baro < t_init) {
        auto baro_msg = baro_->GetLastData();
        pressure.push_back(baro_msg.data);
        CorrectNaN(baro_msg);
      }
    }
  }

  states_.b = CalculateInitialRotationMatrix(g, m);

  auto baro_mean = atlas::Mean(pressure);
  double depth0 = 0;
  if (baro_mean > surface_pressure) {
    // positive depth (underwater)
    auto rho_water = 1000;  // [kh/m3]
    // Saunder-Fofonoff equation
    depth0 = (baro_mean - surface_pressure) / (rho_water * ge_);
  } else {
    // negative depth (outOfWater)
    auto h_s = 0;
    auto p_s = surface_pressure / 100;  // Pressure in mbar
    auto t_s = air_temperature;         // Temperature in Kelvin
    auto p_b = baro_mean / 100;         // Pressure in mbar

    auto k_t = 0.0065;  // K/m
    auto g0 = 9.8065;   // m/s^2
    auto R = 287.1;     // J/kg/K

    depth0 = -t_s / k_t * (std::pow(p_b / p_s, -R * k_t / g0) - 1) + h_s;
  }

  if (depth0 > 0) {
    extra_states_.r_b_dvl(0, 0) = std::cos(heading_shift_dvl);
    extra_states_.r_b_dvl(0, 1) = -std::sin(heading_shift_dvl);
    extra_states_.r_b_dvl(0, 2) = 0;
    extra_states_.r_b_dvl(1, 0) = std::sin(heading_shift_dvl);
    extra_states_.r_b_dvl(1, 1) = std::cos(heading_shift_dvl);
    extra_states_.r_b_dvl(1, 2) = 0;
    extra_states_.r_b_dvl(2, 0) = 0;
    extra_states_.r_b_dvl(2, 1) = 0;
    extra_states_.r_b_dvl(2, 2) = 1;

    auto vel_dvl =
        Eigen::Vector3d(std::get<0>(vel).back(), std::get<1>(vel).back(),
                        std::get<2>(vel).back());

    states_.vel_n =
        extra_states_.r_b_n * extra_states_.r_b_dvl.adjoint() * vel_dvl;
  } else {
    states_.vel_n = Eigen::Vector3d(0, 0, 0);
  }

  states_.pos_n = Eigen::Vector3d(0, 0, depth0);
  states_.acc_bias = Eigen::Vector3d(0, 0, 0);
  states_.gyro_bias = Eigen::Vector3d(0, 0, 0);
  states_.baro_bias = 0;

  kalman_matrix_.qc_ = Eigen::Matrix<double, 13, 13>::Zero(13, 13);
  kalman_matrix_.qc_(0, 0) = sigma_meas_acc;
  kalman_matrix_.qc_(1, 1) = sigma_meas_acc;
  kalman_matrix_.qc_(2, 2) = sigma_meas_acc;
  kalman_matrix_.qc_(3, 3) = sigma_meas_gyr;
  kalman_matrix_.qc_(4, 4) = sigma_meas_gyr;
  kalman_matrix_.qc_(5, 5) = sigma_meas_gyr;
  kalman_matrix_.qc_(6, 6) = sigma_walk_bias_acc;
  kalman_matrix_.qc_(7, 7) = sigma_walk_bias_acc;
  kalman_matrix_.qc_(8, 8) = sigma_walk_bias_acc;
  kalman_matrix_.qc_(9, 9) = sigma_walk_bias_gyr;
  kalman_matrix_.qc_(10, 10) = sigma_walk_bias_gyr;
  kalman_matrix_.qc_(11, 11) = sigma_walk_bias_gyr;
  kalman_matrix_.qc_(12, 12) = sigma_walk_bias_baro;

  kalman_matrix_.p_ = Eigen::Matrix<double, 16, 16>::Zero(16, 16);
  kalman_matrix_.p_(0, 0) = sigma0_pos_x;
  kalman_matrix_.p_(1, 1) = sigma0_pos_y;
  kalman_matrix_.p_(2, 2) = sigma0_pos_z;
  kalman_matrix_.p_(3, 3) = sigma0_vel_x;
  kalman_matrix_.p_(4, 4) = sigma0_vel_y;
  kalman_matrix_.p_(5, 5) = sigma0_vel_z;
  kalman_matrix_.p_(6, 6) = sigma0_rho_x;
  kalman_matrix_.p_(7, 7) = sigma0_rho_y;
  kalman_matrix_.p_(8, 8) = sigma0_rho_z;
  kalman_matrix_.p_(9, 9) = sigma0_bias_acc;
  kalman_matrix_.p_(10, 10) = sigma0_bias_acc;
  kalman_matrix_.p_(11, 11) = sigma0_bias_acc;
  kalman_matrix_.p_(12, 12) = sigma0_bias_gyr;
  kalman_matrix_.p_(13, 13) = sigma0_bias_gyr;
  kalman_matrix_.p_(14, 14) = sigma0_bias_gyr;
  kalman_matrix_.p_(15, 15) = sigma0_bias_baro;
  Notify();
}

//------------------------------------------------------------------------------
//
Eigen::Quaterniond ExtendedKalmanFilter::CalculateInitialRotationMatrix(
    const std::array<std::vector<double>, 3> &g,
    const std::array<std::vector<double>, 3> &m) ATLAS_NOEXCEPT {
  // WARN: Attention should be paid to the values of this vector.
  // If the IMU is inverted, there will be a -1 factor.
  Eigen::Vector3d g_mean;
  g_mean(0) = -imu_sign_x * atlas::Mean(std::get<0>(g));
  g_mean(1) = -imu_sign_y * atlas::Mean(std::get<1>(g));
  g_mean(2) = -imu_sign_z * atlas::Mean(std::get<2>(g));

  if (manual_gravity) {
    ge_ = gravity;
  } else {
    ge_ = g_mean.norm();
  }
  g_n_ = Eigen::Vector3d(0, 0, ge_);

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
  m_b(1) = mag_sign_y * atlas::Mean(std::get<1>(m));
  m_b(2) = mag_sign_z * atlas::Mean(std::get<2>(m));

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

  extra_states_.r_b_n = r0_bn;
  extra_states_.r_n_b = r0_bn.adjoint();
  Eigen::Quaterniond q0 = atlas::RotToQuat(extra_states_.r_n_b);
  extra_states_.euler = atlas::QuatToEuler(q0);

  // The eigen quaternion is taking a rotation matrix as constructor parameter
  // for its initilization.
  return q0;
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::Run() {
  Initialize();

  while (IsRunning()) {
    if (imu_->IsNewDataReady()) {
      ++i_;
      auto imu_msg = imu_->GetLastData();
      double dt = 0;
      dt = imu_->GetDeltaTime();
      imu_timer_.Reset();
      imu_timer_.Start();

      Eigen::Vector3d acc_raw_data;
      acc_raw_data(0) = imu_sign_x * imu_msg.linear_acceleration.x;
      acc_raw_data(1) = imu_sign_y * imu_msg.linear_acceleration.y;
      acc_raw_data(2) = imu_sign_z * imu_msg.linear_acceleration.z;

      Eigen::Vector3d f_b = acc_raw_data - states_.acc_bias;

      Eigen::Vector3d gyr_raw_data;
      gyr_raw_data(0) = imu_sign_x * imu_msg.angular_velocity.x;
      gyr_raw_data(1) = imu_sign_y * imu_msg.angular_velocity.y;
      gyr_raw_data(2) = imu_sign_z * imu_msg.angular_velocity.z;

      extra_states_.w_ib_b = gyr_raw_data - states_.gyro_bias;
      criterions_.ufw = std::pow(extra_states_.w_ib_b.norm(), 2);

      states_.b = atlas::ExactQuat(extra_states_.w_ib_b, dt, states_.b);
      extra_states_.r_n_b = Eigen::Matrix3d(states_.b);
      extra_states_.r_b_n = extra_states_.r_n_b.adjoint();
      extra_states_.euler = atlas::QuatToEuler(states_.b);

      // Propagation step

      Mechanization(f_b, dt);

      ErrorsDynamicModelCalculation();

      KalmanStatesCovariancePropagation(dt);

      Eigen::Vector3d a_b = extra_states_.r_b_n * f_b + g_n_;
      criterions_.ufab = a_b.norm();
      criterions_.ufan = f_b.norm() - ge_;

      if (criterions_.ufab < crit_station_acc &&
          criterions_.ufan < crit_station_norm) {
        is_stationnary_ = true;
      } else {
        is_stationnary_ = false;
      }

      imu_timer_.Pause();

      // Update step

      if (is_stationnary_ && active_gravity) {
        UpdateGravity(f_b);
      }

      if (mag_->IsNewDataReady() && active_mag) {
        auto mag_msg = mag_->GetLastData();
        mag_timer_.Reset();
        mag_timer_.Start();
        Eigen::Vector3d mag_raw_data;
        mag_raw_data(0) = mag_msg.magnetic_field.x;
        mag_raw_data(1) = mag_msg.magnetic_field.y;
        mag_raw_data(2) = mag_msg.magnetic_field.z;
        UpdateMag(mag_raw_data);
        mag_timer_.Pause();
      }

      if (dvl_->IsNewDataReady() && active_dvl) {
        auto dvl_msg = dvl_->GetLastData();
        dvl_timer_.Reset();
        dvl_timer_.Start();
        Eigen::Vector3d dvl_raw_data;
        dvl_raw_data(0) = dvl_msg.twist.twist.linear.x;
        dvl_raw_data(1) = dvl_msg.twist.twist.linear.y;
        dvl_raw_data(2) = dvl_msg.twist.twist.linear.z;
        UpdateDvl(dvl_raw_data);
        dvl_timer_.Pause();
      }

      if (baro_->IsNewDataReady() && active_baro) {
        baro_timer_.Reset();
        baro_timer_.Start();
        UpdateBaro(baro_->GetLastData().data);
        baro_timer_.Pause();
      }

      // Covariance Matrix Simetrization
      kalman_matrix_.p_ = (kalman_matrix_.p_ + kalman_matrix_.p_.adjoint()) / 2;

      Notify();
    }
  }
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::Mechanization(const Eigen::Vector3d &f_b,
                                         const double &dt) ATLAS_NOEXCEPT {
  Eigen::Vector3d p_dot_n = states_.vel_n;

  Eigen::Vector3d v_dot_n = extra_states_.r_b_n * f_b + g_n_;

  states_.pos_n = states_.pos_n + p_dot_n * dt;
  states_.vel_n = states_.vel_n + v_dot_n * dt;
  extra_states_.vel_b = extra_states_.r_n_b * states_.vel_n;
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::ErrorsDynamicModelCalculation() ATLAS_NOEXCEPT {
  static Eigen::Matrix3d f_pv = Eigen::Matrix3d::Identity(3, 3);
  static Eigen::Matrix3d f_vr = atlas::SkewMatrix(g_n_);
  Eigen::Matrix3d f_vbf = -extra_states_.r_b_n;
  Eigen::Matrix3d f_rbg = -extra_states_.r_b_n;

  kalman_matrix_.f_ = Eigen::Matrix<double, 16, 16>::Zero(16, 16);
  kalman_matrix_.f_(0, 3) = f_pv(0, 0);
  kalman_matrix_.f_(0, 4) = f_pv(0, 1);
  kalman_matrix_.f_(0, 5) = f_pv(0, 2);
  kalman_matrix_.f_(1, 3) = f_pv(1, 0);
  kalman_matrix_.f_(1, 4) = f_pv(1, 1);
  kalman_matrix_.f_(1, 5) = f_pv(1, 2);
  kalman_matrix_.f_(2, 3) = f_pv(2, 0);
  kalman_matrix_.f_(2, 4) = f_pv(2, 1);
  kalman_matrix_.f_(2, 5) = f_pv(2, 2);

  kalman_matrix_.f_(3, 6) = f_vr(0, 0);
  kalman_matrix_.f_(3, 7) = f_vr(0, 1);
  kalman_matrix_.f_(3, 8) = f_vr(0, 2);
  kalman_matrix_.f_(4, 6) = f_vr(1, 0);
  kalman_matrix_.f_(4, 7) = f_vr(1, 1);
  kalman_matrix_.f_(4, 8) = f_vr(1, 2);
  kalman_matrix_.f_(5, 6) = f_vr(2, 0);
  kalman_matrix_.f_(5, 7) = f_vr(2, 1);
  kalman_matrix_.f_(5, 8) = f_vr(2, 2);

  kalman_matrix_.f_(3, 9) = f_vbf(0, 0);
  kalman_matrix_.f_(3, 10) = f_vbf(0, 1);
  kalman_matrix_.f_(3, 11) = f_vbf(0, 2);
  kalman_matrix_.f_(4, 9) = f_vbf(1, 0);
  kalman_matrix_.f_(4, 10) = f_vbf(1, 1);
  kalman_matrix_.f_(4, 11) = f_vbf(1, 2);
  kalman_matrix_.f_(5, 9) = f_vbf(2, 0);
  kalman_matrix_.f_(5, 10) = f_vbf(2, 1);
  kalman_matrix_.f_(5, 11) = f_vbf(2, 2);

  kalman_matrix_.f_(6, 12) = f_rbg(0, 0);
  kalman_matrix_.f_(6, 13) = f_rbg(0, 1);
  kalman_matrix_.f_(6, 14) = f_rbg(0, 2);
  kalman_matrix_.f_(7, 12) = f_rbg(1, 0);
  kalman_matrix_.f_(7, 13) = f_rbg(1, 1);
  kalman_matrix_.f_(7, 14) = f_rbg(1, 2);
  kalman_matrix_.f_(8, 12) = f_rbg(2, 0);
  kalman_matrix_.f_(8, 13) = f_rbg(2, 1);
  kalman_matrix_.f_(8, 14) = f_rbg(2, 2);

  Eigen::Matrix3d g_acc = -extra_states_.r_b_n;
  Eigen::Matrix3d g_gyr = -extra_states_.r_b_n;
  Eigen::Matrix3d g_ba = Eigen::Matrix3d::Identity(3, 3);
  Eigen::Matrix3d g_bg = Eigen::Matrix3d::Identity(3, 3);
  static double g_bb = 1;

  kalman_matrix_.g_ = Eigen::Matrix<double, 16, 13>::Zero(16, 13);
  kalman_matrix_.g_(3, 0) = g_acc(0, 0);
  kalman_matrix_.g_(3, 1) = g_acc(0, 1);
  kalman_matrix_.g_(3, 2) = g_acc(0, 2);
  kalman_matrix_.g_(4, 0) = g_acc(1, 0);
  kalman_matrix_.g_(4, 1) = g_acc(1, 1);
  kalman_matrix_.g_(4, 2) = g_acc(1, 2);
  kalman_matrix_.g_(5, 0) = g_acc(2, 0);
  kalman_matrix_.g_(5, 1) = g_acc(2, 1);
  kalman_matrix_.g_(5, 2) = g_acc(2, 2);

  kalman_matrix_.g_(6, 3) = g_gyr(0, 0);
  kalman_matrix_.g_(6, 4) = g_gyr(0, 1);
  kalman_matrix_.g_(6, 5) = g_gyr(0, 2);
  kalman_matrix_.g_(7, 3) = g_gyr(1, 0);
  kalman_matrix_.g_(7, 4) = g_gyr(1, 1);
  kalman_matrix_.g_(7, 5) = g_gyr(1, 2);
  kalman_matrix_.g_(8, 3) = g_gyr(2, 0);
  kalman_matrix_.g_(8, 4) = g_gyr(2, 1);
  kalman_matrix_.g_(8, 5) = g_gyr(2, 2);

  kalman_matrix_.g_(9, 6) = g_ba(0, 0);
  kalman_matrix_.g_(9, 7) = g_ba(0, 1);
  kalman_matrix_.g_(9, 8) = g_ba(0, 2);
  kalman_matrix_.g_(10, 6) = g_ba(1, 0);
  kalman_matrix_.g_(10, 7) = g_ba(1, 1);
  kalman_matrix_.g_(10, 8) = g_ba(1, 2);
  kalman_matrix_.g_(11, 6) = g_ba(2, 0);
  kalman_matrix_.g_(11, 7) = g_ba(2, 1);
  kalman_matrix_.g_(11, 8) = g_ba(2, 2);

  kalman_matrix_.g_(12, 9) = g_bg(0, 0);
  kalman_matrix_.g_(12, 10) = g_bg(0, 1);
  kalman_matrix_.g_(12, 11) = g_bg(0, 2);
  kalman_matrix_.g_(13, 9) = g_bg(1, 0);
  kalman_matrix_.g_(13, 10) = g_bg(1, 1);
  kalman_matrix_.g_(13, 11) = g_bg(1, 2);
  kalman_matrix_.g_(14, 9) = g_bg(2, 0);
  kalman_matrix_.g_(14, 10) = g_bg(2, 1);
  kalman_matrix_.g_(14, 11) = g_bg(2, 2);

  kalman_matrix_.g_(15, 12) = g_bb;
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::KalmanStatesCovariancePropagation(const double &dt)
    ATLAS_NOEXCEPT {
  Eigen::Matrix<double, 16, 16> q =
      kalman_matrix_.g_ * kalman_matrix_.qc_ * kalman_matrix_.g_.adjoint();

  Eigen::Matrix<double, 16, 16> q_k = q * dt;
  q_k(0, 0) = q_k(0, 0) + criterions_.ufw * dt;
  q_k(1, 1) = q_k(1, 1) + criterions_.ufw * dt;
  q_k(2, 2) = q_k(2, 2) + criterions_.ufw * dt;

  Eigen::Matrix<double, 16, 16> phi_k =
      Eigen::Matrix<double, 16, 16>::Identity(16, 16) + kalman_matrix_.f_ * dt;

  kalman_matrix_.p_ =
      phi_k * kalman_matrix_.p_ * kalman_matrix_.p_.adjoint() + q_k;
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateGravity(const Eigen::Vector3d &f_b)
    ATLAS_NOEXCEPT {
  Eigen::Matrix3d skew_g_n = atlas::SkewMatrix(g_n_);

  Eigen::Matrix<double, 3, 16> h_gravity = Eigen::Matrix<double, 3, 16>::Zero();
  h_gravity(0, 6) = -skew_g_n(0, 0);
  h_gravity(0, 7) = -skew_g_n(0, 1);
  h_gravity(0, 8) = -skew_g_n(0, 2);
  h_gravity(1, 6) = -skew_g_n(1, 0);
  h_gravity(1, 7) = -skew_g_n(1, 1);
  h_gravity(1, 8) = -skew_g_n(1, 2);
  h_gravity(2, 6) = -skew_g_n(2, 0);
  h_gravity(2, 7) = -skew_g_n(2, 1);
  h_gravity(2, 8) = -skew_g_n(2, 2);

  Eigen::Matrix<double, 3, 3> r_gravity =
      sigma_meas_gravity * sigma_meas_gravity *
      (criterions_.ufab + criterions_.ufan + criterions_.ufw) *
      Eigen::Matrix3d::Identity(3, 3);

  Eigen::Matrix<double, 3, 3> s_gravity =
      h_gravity * kalman_matrix_.p_ * h_gravity.adjoint() + r_gravity;
  Eigen::Matrix<double, 16, 3> k_gravity =
      kalman_matrix_.p_ * h_gravity.adjoint() * s_gravity.inverse();
  kalman_matrix_.p_ = (Eigen::Matrix<double, 16, 16>::Identity(16, 16) -
                       k_gravity * h_gravity) *
                      kalman_matrix_.p_;

  Eigen::Matrix<double, 3, 1> z_gravity_hat = -extra_states_.r_b_n * f_b;
  Eigen::Matrix<double, 3, 1> z_gravity_meas = g_n_;
  Eigen::Matrix<double, 3, 1> d_z_gravity = z_gravity_meas - z_gravity_hat;

  Eigen::Matrix<double, 16, 1> dx_gravity = k_gravity * d_z_gravity;

  UpdateStates(dx_gravity);
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateMag(const Eigen::Vector3d &mag_raw_data)
    ATLAS_NOEXCEPT {
  Eigen::Vector3d m_b = mag_raw_data;

  double roll = extra_states_.euler(0);
  double pitch = extra_states_.euler(1);
  double yaw_hat = extra_states_.euler(2);

  Eigen::Matrix3d r_b2w = Eigen::Matrix3d::Zero(3, 3);
  r_b2w(0, 0) = std::cos(pitch);
  r_b2w(0, 1) = std::sin(pitch) * std::sin(roll);
  r_b2w(0, 2) = std::sin(pitch) * std::cos(roll);
  r_b2w(1, 1) = std::cos(roll);
  r_b2w(1, 2) = -std::sin(roll);
  r_b2w(2, 0) = -std::sin(pitch);
  r_b2w(2, 1) = std::cos(pitch) * std::sin(roll);
  r_b2w(2, 2) = std::cos(pitch) * std::cos(roll);

  Eigen::Vector3d m_w = r_b2w * m_b;
  double yaw_meas = std::atan2(-m_w(2), m_w(1));

  Eigen::Matrix3d omega_t = Eigen::Matrix3d::Zero(3, 3);
  omega_t(0, 0) = std::cos(yaw_hat) * std::cos(pitch);
  omega_t(0, 1) = -std::sin(yaw_hat);
  omega_t(1, 0) = std::sin(yaw_hat) * std::cos(pitch);
  omega_t(1, 1) = std::cos(yaw_hat);
  omega_t(2, 0) = -std::sin(pitch);
  omega_t(2, 2) = 1;

  Eigen::Matrix3d inv_omega_t = omega_t.inverse();

  Eigen::Matrix<double, 1, 16> h_mag =
      Eigen::Matrix<double, 1, 16>::Zero(1, 16);
  h_mag(0, 6) = inv_omega_t(2, 0);
  h_mag(0, 7) = inv_omega_t(2, 1);
  h_mag(0, 8) = inv_omega_t(2, 2);

  double r_mag = sigma_meas_mag * sigma_meas_mag;

  double s_mag = h_mag * kalman_matrix_.p_ * h_mag.adjoint() + r_mag;
  Eigen::Matrix<double, 16, 1> k_mag =
      kalman_matrix_.p_ * h_mag.adjoint() / s_mag;

  double d_z_mag = yaw_meas - yaw_hat;

  if (std::abs(d_z_mag) < M_PI) {
    Eigen::Matrix<double, 16, 1> dx_mag = k_mag * d_z_mag;
    UpdateStates(dx_mag);
    kalman_matrix_.p_ =
        (Eigen::Matrix<double, 16, 16>::Identity(16, 16) - k_mag * h_mag) *
        kalman_matrix_.p_;
  }
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateDvl(const Eigen::Vector3d &dvl_raw_data)
    ATLAS_NOEXCEPT {
  // Avoid corrections from NaN replaced by zeros
  if (dvl_raw_data.norm() > 0) {
    static Eigen::Matrix<double, 3, 3> r_b_dvl_adj =
        extra_states_.r_b_dvl.adjoint();
    Eigen::Vector3d dvl_data = r_b_dvl_adj * dvl_raw_data;
    ROS_INFO_STREAM(dvl_data(0) << "    " << dvl_data(1) << "    "
                                << dvl_data(2));
    // Aiding Measurement Model
    auto skew_l_pd = atlas::SkewMatrix(l_pd);
    Eigen::Matrix<double, 3, 16> h_dvl = Eigen::Matrix<double, 3, 16>::Zero();
    h_dvl(0, 3) = extra_states_.r_n_b(0, 0);
    h_dvl(0, 4) = extra_states_.r_n_b(0, 1);
    h_dvl(0, 5) = extra_states_.r_n_b(0, 2);
    h_dvl(1, 3) = extra_states_.r_n_b(1, 0);
    h_dvl(1, 4) = extra_states_.r_n_b(1, 1);
    h_dvl(1, 5) = extra_states_.r_n_b(1, 2);
    h_dvl(2, 3) = extra_states_.r_n_b(2, 0);
    h_dvl(2, 4) = extra_states_.r_n_b(2, 1);
    h_dvl(2, 5) = extra_states_.r_n_b(2, 2);
    h_dvl(0, 12) = skew_l_pd(0, 0);
    h_dvl(0, 13) = skew_l_pd(0, 1);
    h_dvl(0, 14) = skew_l_pd(0, 2);
    h_dvl(1, 12) = skew_l_pd(1, 0);
    h_dvl(1, 13) = skew_l_pd(1, 1);
    h_dvl(1, 14) = skew_l_pd(1, 2);
    h_dvl(2, 12) = skew_l_pd(2, 0);
    h_dvl(2, 13) = skew_l_pd(2, 1);
    h_dvl(2, 14) = skew_l_pd(2, 2);

    static Eigen::Matrix3d r_dvl =
        Eigen::Matrix<double, 1, 3>(sigma_meas_dvl_x, sigma_meas_dvl_y,
                                    sigma_meas_dvl_z)
            .asDiagonal();
    Eigen::Matrix<double, 3, 3> s_dvl =
        h_dvl * kalman_matrix_.p_ * h_dvl.adjoint() + r_dvl;
    Eigen::Matrix<double, 16, 3> k_dvl =
        kalman_matrix_.p_ * h_dvl.adjoint() * s_dvl.inverse();
    Eigen::Matrix<double, 16, 16> eye16 = Eigen::Matrix<double, 16, 16>::Zero();
    kalman_matrix_.p_ = (eye16 - k_dvl * h_dvl) * kalman_matrix_.p_;
    // Prediction
    Eigen::Vector3d dvl_hat =
        extra_states_.r_n_b * states_.vel_n + extra_states_.w_ib_b.cross(l_pd);

    Eigen::Vector3d d_z_dvl = dvl_data - dvl_hat;
    Eigen::Matrix<double, 16, 1> d_x_dvl = k_dvl * d_z_dvl;
    UpdateStates(d_x_dvl);
  }
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateBaro(const double &baro_meas) ATLAS_NOEXCEPT {
  double depth_meas = 0;
  static auto rho_water = 1000;  // [kh/m3]

  if (baro_meas > surface_pressure) {
    // positive depth (underwater)
    // Saunder-Fofonoff equation
    depth_meas = (baro_meas - surface_pressure) / (rho_water * ge_);
  } else {
    // negative depth (outOfWater)
    static auto h_s = 0;
    static auto p_s = surface_pressure / 100;  // Pressure in mbar
    static auto t_s = air_temperature;         // Temperature in Kelvin
    static auto p_b = baro_meas / 100;         // Pressure in mbar

    static auto k_t = 0.0065;  // K/m
    static auto g0 = 9.8065;   // m/s^2
    static auto r = 287.1;     // J/kg/K

    depth_meas = -t_s / k_t * (std::pow(p_b / p_s, -r * k_t / g0) - 1) + h_s;
  }

  // Aiding Measurement Model
  auto l_tp = extra_states_.r_n_b * l_pp;
  auto skew_l_tp = atlas::SkewMatrix(l_tp);
  Eigen::Matrix<double, 1, 16> h_baro = Eigen::Matrix<double, 1, 16>::Zero();
  Eigen::Matrix<double, 1, 3> tmp_mat =
      -Eigen::Matrix<double, 1, 3>(0, 0, 1) * skew_l_tp;
  h_baro(0, 2) = 1;
  h_baro(0, 6) = tmp_mat(0);
  h_baro(0, 7) = tmp_mat(1);
  h_baro(0, 8) = tmp_mat(2);
  h_baro(0, 15) = 1 / (rho_water * ge_);

  double r_baro = sigma_meas_baro;
  double depth_hat = states_.pos_n(2);
  double d_z_baro = depth_meas - depth_hat;

  // Apparently canno't do coefficient-wise operation on matrix with Eigen
  // And we need to convert to array first. Lot of type casting here...
  // see: http://eigen.tuxfamily.org/dox-devel/group__TutorialArrayClass.html
  auto k_baro_tmp = h_baro * kalman_matrix_.p_ * h_baro.adjoint();
  auto k_baro_tmp_arr = k_baro_tmp.array() + r_baro;
  auto k_baro =
      kalman_matrix_.p_ * h_baro.adjoint() * k_baro_tmp_arr.matrix().inverse();
  auto d_x = k_baro * d_z_baro;
  UpdateStates(d_x);
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateStates(const Eigen::Matrix<double, 16, 1> &dx)
    ATLAS_NOEXCEPT {
  kalman_states_.d_pos_n(0) = dx(0);
  kalman_states_.d_pos_n(1) = dx(1);
  kalman_states_.d_pos_n(2) = dx(2);
  states_.pos_n += kalman_states_.d_pos_n;

  kalman_states_.d_vel_n(0) = dx(3);
  kalman_states_.d_vel_n(1) = dx(4);
  kalman_states_.d_vel_n(2) = dx(5);
  states_.vel_n = states_.vel_n + kalman_states_.d_vel_n;
  extra_states_.vel_b = extra_states_.r_n_b * states_.vel_n;

  kalman_states_.rho(0) = dx(6);
  kalman_states_.rho(1) = dx(7);
  kalman_states_.rho(2) = dx(8);

  kalman_states_.d_acc_bias(0) = dx(9);
  kalman_states_.d_acc_bias(1) = dx(10);
  kalman_states_.d_acc_bias(2) = dx(11);
  states_.acc_bias += kalman_states_.d_acc_bias;

  kalman_states_.d_gyro_bias(0) = dx(12);
  kalman_states_.d_gyro_bias(1) = dx(13);
  kalman_states_.d_gyro_bias(2) = dx(14);
  states_.gyro_bias += kalman_states_.d_gyro_bias;

  kalman_states_.d_baro_bias = dx(15);
  states_.baro_bias += kalman_states_.d_baro_bias;

  Eigen::Matrix3d r_n_b_tmp = atlas::QuatToRot(states_.b);
  Eigen::Matrix3d r_b_n_tmp = r_n_b_tmp.adjoint();
  Eigen::Matrix3d p_rho = atlas::SkewMatrix(kalman_states_.rho);
  extra_states_.r_b_n = (Eigen::Matrix3d::Identity(3, 3) + p_rho) * r_b_n_tmp;
  extra_states_.r_n_b = extra_states_.r_b_n.adjoint();
  states_.b = atlas::RotToQuat(extra_states_.r_n_b);
  extra_states_.euler = atlas::QuatToEuler(states_.b);
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::CorrectNaN(DvlMessage &msg) const ATLAS_NOEXCEPT {
  ReplaceNaNByZero(msg.twist.twist.angular.x, "twist.angular.x");
  ReplaceNaNByZero(msg.twist.twist.angular.y, "twist.angular.y");
  ReplaceNaNByZero(msg.twist.twist.angular.z, "twist.angular.z");
  ReplaceNaNByZero(msg.twist.twist.linear.x, "twist.linear.x");
  ReplaceNaNByZero(msg.twist.twist.linear.y, "twist.linear.y");
  ReplaceNaNByZero(msg.twist.twist.linear.z, "twist.linear.z");
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::CorrectNaN(ImuMessage &msg) const ATLAS_NOEXCEPT {
  ReplaceNaNByZero(msg.orientation.w, "orientation.w");
  ReplaceNaNByZero(msg.orientation.x, "orientation.x");
  ReplaceNaNByZero(msg.orientation.y, "orientation.y");
  ReplaceNaNByZero(msg.orientation.z, "orientation.z");
  ReplaceNaNByZero(msg.angular_velocity.x, "angular_velocity.x");
  ReplaceNaNByZero(msg.angular_velocity.y, "angular_velocity.y");
  ReplaceNaNByZero(msg.angular_velocity.z, "angular_velocity.z");
  ReplaceNaNByZero(msg.linear_acceleration.x, "angular_velocity.x");
  ReplaceNaNByZero(msg.linear_acceleration.y, "linear_acceleration.y");
  ReplaceNaNByZero(msg.linear_acceleration.z, "linear_acceleration.z");
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::CorrectNaN(MagMessage &msg) const ATLAS_NOEXCEPT {
  ReplaceNaNByZero(msg.magnetic_field.x, "magnetic_field.x");
  ReplaceNaNByZero(msg.magnetic_field.y, "magnetic_field.y");
  ReplaceNaNByZero(msg.magnetic_field.z, "magnetic_field.z");
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::CorrectNaN(BaroMessage &msg) const ATLAS_NOEXCEPT {
  ReplaceNaNByZero(msg.data, "data");
}

}  // namespace proc_navigation
