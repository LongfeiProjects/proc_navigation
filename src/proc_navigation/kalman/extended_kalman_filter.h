/**
 * \file	extended_kalman_filter.h
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

#ifndef PROC_NAVIGATION_KALMAN_EXTENDED_KALMAN_FILTER_H_
#define PROC_NAVIGATION_KALMAN_EXTENDED_KALMAN_FILTER_H_

#include <memory>
#include <vector>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <eigen3/Eigen/Eigen>
#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/subject.h>
#include <lib_atlas/pattern/runnable.h>
#include <lib_atlas/sys/timer.h>
#include "proc_navigation/kalman/ekf_configuration.h"
#include "proc_navigation/kalman/state_controller.h"

namespace proc_navigation {

class ExtendedKalmanFilter : public atlas::Runnable,
                             private EkfConfiguration,
                             public atlas::Subject<> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ExtendedKalmanFilter>;
  using ConstPtr = std::shared_ptr<const ExtendedKalmanFilter>;
  using PtrList = std::vector<ExtendedKalmanFilter::Ptr>;
  using ConstPtrList = std::vector<ExtendedKalmanFilter::ConstPtr>;

  using BaroMessage = std_msgs::Float64;
  using DvlMessage = geometry_msgs::TwistWithCovarianceStamped;
  using ImuMessage = sensor_msgs::Imu;
  using MagMessage = sensor_msgs::MagneticField;

  struct States {
    Eigen::Vector3d pos_n;
    Eigen::Vector3d vel_n;
    Eigen::Quaterniond b;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;
    double baro_bias;
  };

  struct KalmanStates {
    Eigen::Vector3d d_pos_n;
    Eigen::Vector3d d_vel_n;
    Eigen::Vector3d rho;
    Eigen::Vector3d d_acc_bias;
    Eigen::Vector3d d_gyro_bias;
    double d_baro_bias;
  };

  struct ExtraStates {
    Eigen::Matrix3d r_n_b;
    Eigen::Matrix3d r_b_n;
    Eigen::Vector3d euler;  // roll, pitch, yaw
    Eigen::Vector3d w_ib_b;
    Eigen::Vector3d vel_b;
    double baro_bias;
  };

  struct KalmanMatrix {
    Eigen::Matrix<double, 13, 13> qc_;
    Eigen::Matrix<double, 16, 16> p_;
    Eigen::Matrix<double, 16, 16> f_;
    Eigen::Matrix<double, 16, 13> g_;
  };

  struct Criterions {
    double ufw;
    double ufab;
    double ufan;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ExtendedKalmanFilter(const StateController<BaroMessage>::Ptr &baro,
                                const StateController<ImuMessage>::Ptr &imu,
                                const StateController<MagMessage>::Ptr &mag,
                                const StateController<DvlMessage>::Ptr &dvl,
                                const EkfConfiguration &conf) ATLAS_NOEXCEPT;

  ~ExtendedKalmanFilter() ATLAS_NOEXCEPT;

  //==========================================================================
  // P U B L I C   M E T H O D S

  const States &GetStates() const ATLAS_NOEXCEPT;

  const ExtraStates &GetExtraStates() const ATLAS_NOEXCEPT;

  const KalmanStates &GetKalmanStates() const ATLAS_NOEXCEPT;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * The Initiate method will be called at the begginning of the system.
   * It is the method that will set the inital velocity, accel, and variances.
   * The time of the Initiate step is being set in the ekf_constants
   * configuration file (Deserialized in EkFConfiguration).
   */
  void Initialize();

  /**
   * The Run method is the loop of the Kalman Filter. When a new data is ready,
   * this will launch a new instance of the processing loop and compute the
   * odometry data.
   */
  void Run() override;

  Eigen::Quaterniond CalculateInitialRotationMatrix(
      const std::array<std::vector<double>, 3> &g,
      const std::array<std::vector<double>, 3> &m) ATLAS_NOEXCEPT;

  void Mechanization(const Eigen::Vector3d &f_b,
                     const double &dt) ATLAS_NOEXCEPT;

  void ErrorsDynamicModelCalculation() ATLAS_NOEXCEPT;

  void KalmanStatesCovariancePropagation(const double &dt) ATLAS_NOEXCEPT;

  void UpdateGravity(const Eigen::Vector3d &) ATLAS_NOEXCEPT;

  void UpdateMag(const Eigen::Vector3d &) ATLAS_NOEXCEPT;

  void UpdateDvl(const Eigen::Vector3d &) ATLAS_NOEXCEPT;

  void UpdateBaro(const double &) ATLAS_NOEXCEPT;

  void UpdateStates(const Eigen::Matrix<double, 16, 1> &dx) ATLAS_NOEXCEPT;

  void CorrectNaN(DvlMessage &msg) const ATLAS_NOEXCEPT;
  void CorrectNaN(ImuMessage &msg) const ATLAS_NOEXCEPT;
  void CorrectNaN(MagMessage &msg) const ATLAS_NOEXCEPT;
  void CorrectNaN(BaroMessage &msg) const ATLAS_NOEXCEPT;

  template <class Tp_>
  void ReplaceNaNByZero(Tp_ &, const std::string &) const ATLAS_NOEXCEPT;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * State if a new data has came from on of the StateController.
   * The loop will not run if no data is ready.
   */
  StateController<BaroMessage>::Ptr baro_;
  StateController<ImuMessage>::Ptr imu_;
  StateController<MagMessage>::Ptr mag_;
  StateController<DvlMessage>::Ptr dvl_;

  /**
   * This is the timer that run during the init time.
   * The value of this timer is going to be compared to the configuration init
   * time value.
   * When the timer hits the init time, the processing of the parallel thread
   * starts.
   */
  atlas::MicroTimer init_timer_;

  /**
   * The initial state and states of the kalman filter.
   * The values of the states are set after the initialization step.
   */
  States states_;
  KalmanStates kalman_states_;
  ExtraStates extra_states_;
  KalmanMatrix kalman_matrix_;
  Criterions criterions_;

  /*
   * Stationnary and static states
   */
  bool is_stationnary_;

  /**
   * Estimation of the gravitationnal vector with the magnetometer.
   */
  double ge_;

  Eigen::Vector3d g_n_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
ATLAS_INLINE const ExtendedKalmanFilter::States &
ExtendedKalmanFilter::GetStates() const ATLAS_NOEXCEPT {
  return states_;
}

//------------------------------------------------------------------------------
//
ATLAS_INLINE const ExtendedKalmanFilter::KalmanStates &
ExtendedKalmanFilter::GetKalmanStates() const ATLAS_NOEXCEPT {
  return kalman_states_;
}

//------------------------------------------------------------------------------
//
ATLAS_INLINE const ExtendedKalmanFilter::ExtraStates &
ExtendedKalmanFilter::GetExtraStates() const ATLAS_NOEXCEPT {
  return extra_states_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE void ExtendedKalmanFilter::ReplaceNaNByZero(
    Tp_ &val, const std::string &str) const ATLAS_NOEXCEPT {
  if (std::isnan(val)) {
    ROS_WARN_STREAM("Received a NaN on the " << str
                                             << " value, replacing by 0.");
    val = 0;
  }
}

}  // namespace proc_navigation

#endif  // PROC_NAVIGATION_KALMAN_EXTENDED_KALMAN_FILTER_H_
