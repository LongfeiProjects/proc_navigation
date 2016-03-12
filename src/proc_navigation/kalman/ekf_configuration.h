/**
 * \file	ekf_configuration.h
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

#ifndef PROC_NAVIGATION_KALMAN_EKF_CONFIGURATION_H_
#define PROC_NAVIGATION_KALMAN_EKF_CONFIGURATION_H_

#include <memory>
#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <lib_atlas/macros.h>

namespace proc_navigation {

class EkfConfiguration {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<EkfConfiguration>;
  using ConstPtr = std::shared_ptr<const EkfConfiguration>;
  using PtrList = std::vector<EkfConfiguration::Ptr>;
  using ConstPtrList = std::vector<EkfConfiguration::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit EkfConfiguration(const ros::NodeHandle &nh) ATLAS_NOEXCEPT;

  /**
   * We want to define copy constructor here for optimization purpose.
   * Do not copy the members if is a rvalue.
   */
  explicit EkfConfiguration(const EkfConfiguration &rhs) ATLAS_NOEXCEPT;
  explicit EkfConfiguration(EkfConfiguration &&rhs) ATLAS_NOEXCEPT;

  virtual ~EkfConfiguration() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E M B E R S

  // The constants for the tuning of the Kalman filter
  double t_init;
  bool manual_gravity;
  double gravity;
  double heading_shift_dvl;
  double air_temperature;
  double surface_pressure;
  bool active_gravity;
  bool active_mag;
  bool active_dvl;
  bool active_baro;
  double sigma_meas_gravity;
  double sigma_meas_mag;
  double sigma_meas_dvl_x;
  double sigma_meas_dvl_y;
  double sigma_meas_dvl_z;
  double sigma_meas_baro;
  double sigma0_pos_x;
  double sigma0_pos_y;
  double sigma0_pos_z;
  double sigma0_vel_x;
  double sigma0_vel_y;
  double sigma0_vel_z;
  double sigma0_rho_x;
  double sigma0_rho_y;
  double sigma0_rho_z;
  double sigma0_bias_acc;
  double sigma0_bias_gyr;
  double sigma0_bias_baro;
  double sigma_meas_acc;
  double sigma_meas_gyr;
  double sigma_walk_bias_acc;
  double sigma_walk_bias_gyr;
  double sigma_walk_bias_baro;
  Eigen::Vector3d l_pd;
  Eigen::Vector3d l_pp;
  double crit_station_acc;
  double crit_station_norm;

  // Values of the coefficients given the orientation of the devices
  int imu_sign_x;
  int imu_sign_y;
  int imu_sign_z;
  int mag_sign_x;
  int mag_sign_y;
  int mag_sign_z;

  // Values for the topics
  std::string baro_topic;
  std::string dvl_topic;
  std::string imu_topic;
  std::string mag_topic;

  // Members for the simulation
  bool simulation_active;
  double simuation_dt_imu;
  double simulation_dt_mag;
  double simulation_dt_dvl;
  double simulation_dt_baro;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  void DeserializeConfiguration() ATLAS_NOEXCEPT;

  template <typename Tp_>
  void FindParameter(const std::string &str, Tp_ &p) ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandle nh_;
};

}  // namespace proc_navigation

#endif  // PROC_NAVIGATION_KALMAN_EKF_CONFIGURATION_H_
