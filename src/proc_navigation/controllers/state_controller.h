/**
 * \file	state_controller.h
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

#ifndef PROC_NAVIGATION_CONTROLLER_STATE_CONTROLLER_H_
#define PROC_NAVIGATION_CONTROLLER_STATE_CONTROLLER_H_

#include <memory>
#include <vector>
#include <mutex>
#include <atomic>
#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/subject.h>
#include <lib_atlas/pattern/observer.h>
#include <lib_atlas/sys/timer.h>
#include <ros/node_handle.h>

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
class StateController : public atlas::Subject<> {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<StateController<Tp_>>;
  using ConstPtr = std::shared_ptr<const StateController<Tp_>>;
  using PtrList = std::vector<StateController::Ptr>;
  using ConstPtrList = std::vector<StateController::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  StateController(const ros::NodeHandlePtr &) ATLAS_NOEXCEPT;

  virtual ~StateController() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E T H O D S

  bool IsNewDataReady() const ATLAS_NOEXCEPT;

 protected:
  //============================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * This accessor returns the time between two measurments.
   * The method is set protected to prevent the user to access the delta_t.
   * If the user access the delta_t that is not associated with its current
   * data, it will create unwanted behavior. Prefer instead creating a structure
   * of the information you want the user to have and send it when you have a
   * new information available.
   */
  double GetTimeDelta() const ATLAS_NOEXCEPT;

  /**
   * This method will handle the behavior of a StateController when a new data
   * comes. It will update the elapsed time for the data and alert the
   * observers.
   * The parsing of the data has to be done by children
   */
  void ReceivedNewData() ATLAS_NOEXCEPT;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  //============================================================================
  // P R I V A T E   M E M B E R S

  std::mutex time_mutex_;
  double delta_t_;
  atlas::MicroTimer timer;

  std::atomic<bool> new_data_ready_;
};

}  // namespace proc_navigation

#endif  // PROC_NAVIGATION_CONTROLLER_STATE_CONTROLLER_H_
