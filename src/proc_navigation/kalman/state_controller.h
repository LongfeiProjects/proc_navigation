/**
 * \file	state_controller.h
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

#ifndef PROC_NAVIGATION_CONTROLLER_STATE_CONTROLLER_H_
#define PROC_NAVIGATION_CONTROLLER_STATE_CONTROLLER_H_

#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/observer.h>
#include <lib_atlas/pattern/subject.h>
#include <lib_atlas/sys/timer.h>
#include <ros/node_handle.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

namespace proc_navigation {

/**
 * A state controller is an object that will feed the Kalman filter with new
 * data. It is being subscribe to a source of data and will estimate the time
 * between the two measurments.
 * This is very important as it is the object that provides the dt to the
 * Kalman filter.
 *
 * Note: This is notifying the observer with no data type as the data type will
 * depend on the child. No template usage is requiered here as the observer
 * will have to dynamic cast the message anyway...
 */
template <class Tp_>
class StateController {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<StateController<Tp_>>;
  using ConstPtr = std::shared_ptr<const StateController<Tp_>>;
  using PtrList = std::vector<StateController::Ptr>;
  using ConstPtrList = std::vector<StateController::ConstPtr>;

  using DataType = typename Tp_::Ptr;

  static const uint32_t BUFFER_SIZE;

  //============================================================================
  // P U B L I C   C / D T O R S

  /**
   * Receiving the node handle as well as the configuration name for the topic
   * (NOT the topic name itself)
   */
  explicit StateController(const ros::NodeHandle &,
                           const std::string &conf_name,
                           bool simulation = false,
                           double sim_dt = 0) ATLAS_NOEXCEPT;

  virtual ~StateController() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E T H O D S

  bool IsNewDataReady() const ATLAS_NOEXCEPT;

  DataType GetLastData();

  DataType GetLastDataIfDtIn(double dt);

  void Callback(const DataType &msg) ATLAS_NOEXCEPT;

  double GetDeltaTime() const ATLAS_NOEXCEPT;

  /**
   * Return the delta time between the last message received and the first one.
   */
  double GetTimeForLast() const ATLAS_NOEXCEPT;
  double GetTimeForCurrent() const ATLAS_NOEXCEPT;
  double GetTimeForNext() const ATLAS_NOEXCEPT;

  uint64_t GetMessageCount() const ATLAS_NOEXCEPT;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  /**
   * The delta time is the difference of the stamp in the header of two
   * consecutive packages. It highly depend on the implementation of the
   * package that sends the message and this may not be functionnal.
   */
  double GetStampedDeltaTime() const ATLAS_NOEXCEPT;

  double GetSimulatedDeltaTime() const ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandle nh_;

  mutable std::mutex data_mutex_;
  std::queue<DataType> data_buffer_;
  DataType current_data_;

  std::atomic<bool> new_data_ready_;
  std::atomic<bool> is_simulated_time_;

  double first_stamped_t_;
  double last_stamped_t_;
  double current_stamped_t_;
  double stamped_dt_;
  double sim_dt_;

  uint64_t message_count_;

  ros::Subscriber subscriber_;
};

}  // namespace proc_navigation

#include "proc_navigation/kalman/state_controller_inl.h"

#endif  // PROC_NAVIGATION_CONTROLLER_STATE_CONTROLLER_H_
