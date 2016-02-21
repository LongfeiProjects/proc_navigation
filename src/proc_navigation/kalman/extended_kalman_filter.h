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
#include <eigen3/Eigen/Eigen>
#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/subject.h>
#include <lib_atlas/pattern/runnable.h>
#include <lib_atlas/sys/timer.h>
#include "proc_navigation/kalman/ekf_configuration.h"

namespace proc_navigation {

class ExtendedKalmanFilter : public atlas::Observer<>,
                             public atlas::Runnable,
                             private EkfConfiguration {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ExtendedKalmanFilter>;
  using ConstPtr = std::shared_ptr<const ExtendedKalmanFilter>;
  using PtrList = std::vector<ExtendedKalmanFilter::Ptr>;
  using ConstPtrList = std::vector<ExtendedKalmanFilter::ConstPtr>;

  struct InitialState {
    double ge;
    double roll;
    double pitch;
    double yaw;
    Eigen::Matrix3d r_b2w;
    Eigen::Matrix3d r0_bn;
    Eigen::Matrix3d r0_nb;
    Eigen::Quaterniond b0;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ExtendedKalmanFilter(const EkfConfiguration &conf) ATLAS_NOEXCEPT;

  ~ExtendedKalmanFilter() ATLAS_NOEXCEPT;

  //==========================================================================
  // P U B L I C   M E T H O D S

  void UpdateImuData() ATLAS_NOEXCEPT;
  void UpdateDvlData() ATLAS_NOEXCEPT;
  void UpdateBaroData() ATLAS_NOEXCEPT;

  bool IsNewDataReady() const ATLAS_NOEXCEPT;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * The Initiate method will be called at the begginning of the system.
   * It is the method that will set the inital velocity, accel, and variances.
   * The time of the Initiate step is being set in the ekf_constants
   * configuration file (Deserialized in EkFConfiguration).
   */
  void Initiate();

  /**
   * The Run method is the loop of the Kalman Filter. When a new data is ready,
   * this will launch a new instance of the processing loop and compute the
   * odometry data.
   */
  void Run() override;

  void OnSubjectNotify(atlas::Subject<> &subject) ATLAS_NOEXCEPT override;

  void CalculateImuMeans(const std::array<std::vector<double>, 3> &g)
      ATLAS_NOEXCEPT;

  void CalculateMagMeans(const std::array<std::vector<double>, 3> &m)
      ATLAS_NOEXCEPT;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * State if a new data has came from on of the StateController.
   * The loop will not run if no data is ready.
   */
  std::atomic<bool> new_data_ready_;

  /**
   * This is the timer that run during the init time.
   * The value of this timer is going to be compared to the configuration init
   * time value.
   * When the timer hits the init time, the processing of the parallel thread
   * starts.
   */
  atlas::MicroTimer init_timer_;

  InitialState init_;

  /**
   * As we don't want to access the data if a proccessing loop instance is
   * occuring, we will simply block th access to the data at any time during
   * processing loop.
   */
  std::mutex processing_mutex_;
};

}  // namespace proc_navigation

#endif  // PROC_NAVIGATION_KALMAN_EXTENDED_KALMAN_FILTER_H_
