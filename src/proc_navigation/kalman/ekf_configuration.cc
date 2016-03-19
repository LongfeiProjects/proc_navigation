/**
 * \file	ekf_configuration.cc
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

#include "proc_navigation/kalman/ekf_configuration.h"

namespace proc_navigation {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
EkfConfiguration::EkfConfiguration(const ros::NodeHandle &nh) ATLAS_NOEXCEPT
    : t_init(0.5),
      manual_gravity(true),
      gravity(9.8),
      heading_shift_dvl(-45),
      air_temperature(298.15),
      surface_pressure(101325),
      active_gravity(true),
      active_mag(true),
      active_dvl(true),
      active_baro(true),
      sigma_meas_gravity(0.010),
      sigma_meas_mag(0.010),
      sigma_meas_dvl_x(0.10),
      sigma_meas_dvl_y(0.10),
      sigma_meas_dvl_z(0.10),
      sigma_meas_baro(0.010),
      sigma0_pos_x(0.01),
      sigma0_pos_y(0.01),
      sigma0_pos_z(0.01),
      sigma0_vel_x(0.01),
      sigma0_vel_y(0.01),
      sigma0_vel_z(0.01),
      sigma0_rho_x(0.01),
      sigma0_rho_y(0.01),
      sigma0_rho_z(0.01),
      sigma0_bias_acc(0.1),
      sigma0_bias_gyr(0.001),
      sigma0_bias_baro(0.001),
      sigma_meas_acc(15.),
      sigma_meas_gyr(0.01),
      sigma_walk_bias_acc(0.001),
      sigma_walk_bias_gyr(0.001),
      sigma_walk_bias_baro(0.001),
      l_pd({0.0, 0.0, 0.15}),
      l_pp({0.0, -0.10f, -0.5f}),
      crit_station_acc(1.0),
      crit_station_norm(1.0),
      imu_sign_x(1),
      imu_sign_y(-1),
      imu_sign_z(-1),
      mag_sign_x(1),
      mag_sign_y(-1),
      mag_sign_z(-1),
      baro_topic("/auv6/pressure"),
      dvl_topic("/provider_dvl/twist"),
      imu_topic("/provider_imu/imu"),
      mag_topic("/provider_imu/magnetic_field"),
      simulation_active(false),
      simulation_dt_imu(0.01),
      simulation_dt_mag(0.01),
      simulation_dt_dvl(0.2857142857142857),
      simulation_dt_baro(0.072),
      nh_(nh) {
  DeserializeConfiguration();
}

//------------------------------------------------------------------------------
//
EkfConfiguration::EkfConfiguration(const EkfConfiguration &rhs) ATLAS_NOEXCEPT {
  t_init = rhs.t_init;
  manual_gravity = rhs.manual_gravity;
  gravity = rhs.gravity;
  heading_shift_dvl = rhs.heading_shift_dvl;
  air_temperature = rhs.air_temperature;
  surface_pressure = rhs.surface_pressure;
  active_gravity = rhs.active_gravity;
  active_mag = rhs.active_mag;
  active_dvl = rhs.active_dvl;
  active_baro = rhs.active_baro;
  sigma_meas_gravity = rhs.sigma_meas_gravity;
  sigma_meas_mag = rhs.sigma_meas_mag;
  sigma_meas_dvl_x = rhs.sigma_meas_dvl_x;
  sigma_meas_dvl_y = rhs.sigma_meas_dvl_y;
  sigma_meas_dvl_z = rhs.sigma_meas_dvl_z;
  sigma_meas_baro = rhs.sigma_meas_baro;
  sigma0_pos_x = rhs.sigma0_pos_x;
  sigma0_pos_y = rhs.sigma0_pos_y;
  sigma0_pos_z = rhs.sigma0_pos_z;
  sigma0_vel_x = rhs.sigma0_vel_x;
  sigma0_vel_y = rhs.sigma0_vel_y;
  sigma0_vel_z = rhs.sigma0_vel_z;
  sigma0_rho_x = rhs.sigma0_rho_x;
  sigma0_rho_y = rhs.sigma0_rho_y;
  sigma0_rho_z = rhs.sigma0_rho_z;
  sigma0_bias_acc = rhs.sigma0_bias_acc;
  sigma0_bias_gyr = rhs.sigma0_bias_gyr;
  sigma0_bias_baro = rhs.sigma0_bias_baro;
  sigma_meas_acc = rhs.sigma_meas_acc;
  sigma_meas_gyr = rhs.sigma_meas_gyr;
  sigma_walk_bias_acc = rhs.sigma_walk_bias_acc;
  sigma_walk_bias_gyr = rhs.sigma_walk_bias_gyr;
  sigma_walk_bias_baro = rhs.sigma_walk_bias_baro;
  l_pd = rhs.l_pd;
  l_pp = rhs.l_pp;
  crit_station_acc = rhs.crit_station_acc;
  crit_station_norm = rhs.crit_station_norm;
  imu_sign_x = rhs.imu_sign_x;
  imu_sign_y = rhs.imu_sign_y;
  imu_sign_z = rhs.imu_sign_z;
  mag_sign_x = rhs.mag_sign_x;
  mag_sign_y = rhs.mag_sign_y;
  mag_sign_z = rhs.mag_sign_z;
  simulation_active = rhs.simulation_active;
  simulation_dt_imu = rhs.simulation_dt_imu;
  simulation_dt_mag = rhs.simulation_dt_mag;
  simulation_dt_dvl = rhs.simulation_dt_dvl;
  simulation_dt_baro = rhs.simulation_dt_baro;
  nh_ = rhs.nh_;
}

//------------------------------------------------------------------------------
//
EkfConfiguration::EkfConfiguration(EkfConfiguration &&rhs) ATLAS_NOEXCEPT {
  t_init = rhs.t_init;
  manual_gravity = rhs.manual_gravity;
  gravity = rhs.gravity;
  heading_shift_dvl = rhs.heading_shift_dvl;
  air_temperature = rhs.air_temperature;
  surface_pressure = rhs.surface_pressure;
  active_gravity = rhs.active_gravity;
  active_mag = rhs.active_mag;
  active_dvl = rhs.active_dvl;
  active_baro = rhs.active_baro;
  sigma_meas_gravity = rhs.sigma_meas_gravity;
  sigma_meas_mag = rhs.sigma_meas_mag;
  sigma_meas_dvl_x = rhs.sigma_meas_dvl_x;
  sigma_meas_dvl_y = rhs.sigma_meas_dvl_y;
  sigma_meas_dvl_z = rhs.sigma_meas_dvl_z;
  sigma_meas_baro = rhs.sigma_meas_baro;
  sigma0_pos_x = rhs.sigma0_pos_x;
  sigma0_pos_y = rhs.sigma0_pos_y;
  sigma0_pos_z = rhs.sigma0_pos_z;
  sigma0_vel_x = rhs.sigma0_vel_x;
  sigma0_vel_y = rhs.sigma0_vel_y;
  sigma0_vel_z = rhs.sigma0_vel_z;
  sigma0_rho_x = rhs.sigma0_rho_x;
  sigma0_rho_y = rhs.sigma0_rho_y;
  sigma0_rho_z = rhs.sigma0_rho_z;
  sigma0_bias_acc = rhs.sigma0_bias_acc;
  sigma0_bias_gyr = rhs.sigma0_bias_gyr;
  sigma0_bias_baro = rhs.sigma0_bias_baro;
  sigma_meas_acc = rhs.sigma_meas_acc;
  sigma_meas_gyr = rhs.sigma_meas_gyr;
  sigma_walk_bias_acc = rhs.sigma_walk_bias_acc;
  sigma_walk_bias_gyr = rhs.sigma_walk_bias_gyr;
  sigma_walk_bias_baro = rhs.sigma_walk_bias_baro;
  l_pd = rhs.l_pd;
  l_pp = rhs.l_pp;
  crit_station_acc = rhs.crit_station_acc;
  crit_station_norm = rhs.crit_station_norm;
  imu_sign_x = rhs.imu_sign_x;
  imu_sign_y = rhs.imu_sign_y;
  imu_sign_z = rhs.imu_sign_z;
  mag_sign_x = rhs.mag_sign_x;
  mag_sign_y = rhs.mag_sign_y;
  mag_sign_z = rhs.mag_sign_z;
  simulation_active = rhs.simulation_active;
  simulation_dt_imu = rhs.simulation_dt_imu;
  simulation_dt_mag = rhs.simulation_dt_mag;
  simulation_dt_dvl = rhs.simulation_dt_dvl;
  simulation_dt_baro = rhs.simulation_dt_baro;
  nh_ = rhs.nh_;
}

//------------------------------------------------------------------------------
//
EkfConfiguration::~EkfConfiguration() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void EkfConfiguration::DeserializeConfiguration() ATLAS_NOEXCEPT {
  FindParameter("/ekf/t_init", t_init);
  FindParameter("/ekf/manual_gravity", manual_gravity);
  FindParameter("/ekf/gravity", gravity);
  FindParameter("/ekf/heading_shift_dvl", heading_shift_dvl);
  FindParameter("/ekf/air_temperature", air_temperature);
  FindParameter("/ekf/surface_pressure", surface_pressure);
  FindParameter("/ekf/active_gravity", active_gravity);
  FindParameter("/ekf/active_mag", active_mag);
  FindParameter("/ekf/active_dvl", active_dvl);
  FindParameter("/ekf/active_baro", active_baro);
  FindParameter("/ekf/sigma_meas_gravity", sigma_meas_gravity);
  FindParameter("/ekf/sigma_meas_mag", sigma_meas_mag);
  FindParameter("/ekf/sigma_meas_dvl_x", sigma_meas_dvl_x);
  FindParameter("/ekf/sigma_meas_dvl_y", sigma_meas_dvl_y);
  FindParameter("/ekf/sigma_meas_dvl_z", sigma_meas_dvl_z);
  FindParameter("/ekf/sigma_meas_baro", sigma_meas_baro);
  FindParameter("/ekf/sigma0_pos_x", sigma0_pos_x);
  FindParameter("/ekf/sigma0_pos_y", sigma0_pos_y);
  FindParameter("/ekf/sigma0_pos_z", sigma0_pos_z);
  FindParameter("/ekf/sigma0_vel_x", sigma0_vel_x);
  FindParameter("/ekf/sigma0_vel_y", sigma0_vel_y);
  FindParameter("/ekf/sigma0_vel_z", sigma0_vel_z);
  FindParameter("/ekf/sigma0_rho_x", sigma0_rho_x);
  FindParameter("/ekf/sigma0_rho_y", sigma0_rho_y);
  FindParameter("/ekf/sigma0_rho_z", sigma0_rho_z);
  FindParameter("/ekf/sigma0_bias_acc", sigma0_bias_acc);
  FindParameter("/ekf/sigma0_bias_gyr", sigma0_bias_gyr);
  FindParameter("/ekf/sigma0_bias_baro", sigma0_bias_baro);
  FindParameter("/ekf/sigma_meas_acc", sigma_meas_acc);
  FindParameter("/ekf/sigma_meas_gyr", sigma_meas_gyr);
  FindParameter("/ekf/sigma_walk_bias_acc", sigma_walk_bias_acc);
  FindParameter("/ekf/sigma_walk_bias_gyr", sigma_walk_bias_gyr);
  FindParameter("/ekf/sigma_walk_bias_baro", sigma_walk_bias_baro);
  FindParameter("/ekf/crit_station_acc", crit_station_acc);
  FindParameter("/ekf/crit_station_norm", crit_station_norm);
  FindParameter("/device_sign/imu/x", imu_sign_x);
  FindParameter("/device_sign/imu/y", imu_sign_y);
  FindParameter("/device_sign/imu/z", imu_sign_z);
  FindParameter("/device_sign/mag/x", mag_sign_x);
  FindParameter("/device_sign/mag/y", mag_sign_y);
  FindParameter("/device_sign/mag/z", mag_sign_z);

  FindParameter("/proc_navigation/simulation/active", simulation_active);
  FindParameter("/proc_navigation/simulation/dt_imu", simulation_dt_imu);
  FindParameter("/proc_navigation/simulation/dt_mag", simulation_dt_mag);
  FindParameter("/proc_navigation/simulation/dt_dvl", simulation_dt_dvl);
  FindParameter("/proc_navigation/simulation/dt_baro", simulation_dt_baro);

  // Getting the matrix for Eigen compatible types
  std::vector<double> l_pd_tmp({static_cast<double>(l_pd(0)),
                                static_cast<double>(l_pd(1)),
                                static_cast<double>(l_pd(2))});
  std::vector<double> l_pp_tmp({static_cast<double>(l_pp(0)),
                                static_cast<double>(l_pp(1)),
                                static_cast<double>(l_pp(2))});
  FindParameter("/ekf/l_pd", l_pd_tmp);
  FindParameter("/ekf/l_pp", l_pp_tmp);
  if (l_pd_tmp.size() != 3 || l_pp_tmp.size() != 3) {
    throw std::runtime_error(
        "The parameter l_pd and l_pp must be 3 dim vectors");
  }
  l_pd = Eigen::Vector3d(l_pd_tmp[0], l_pd_tmp[1], l_pd_tmp[2]);
  l_pp = Eigen::Vector3d(l_pp_tmp[0], l_pp_tmp[1], l_pp_tmp[2]);

  // Converting the value of the pressure in rad
  heading_shift_dvl = static_cast<double>(heading_shift_dvl * M_PI / 180);
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
void EkfConfiguration::FindParameter(const std::string &str,
                                     Tp_ &p) ATLAS_NOEXCEPT {
  if (nh_.hasParam("/proc_navigation" + str)) {
    nh_.getParam("/proc_navigation" + str, p);
  } else {
    ROS_WARN_STREAM("Did not find /proc_navigation" + str
                    << ". Using default value instead.");
  }
}

}  // namespace proc_navigation
