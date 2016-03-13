/**
 * \file	state_controller.cc
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
#error This file may only be included state_controller.h
#endif  // PROC_NAVIGATION_CONTROLLER_STATE_CONTROLLER_H_

#include "proc_navigation/kalman/state_controller.h"

namespace proc_navigation {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE StateController<Tp_>::StateController(
    const ros::NodeHandle &nh, const std::string &topic_name, bool simulation,
    double sim_dt) ATLAS_NOEXCEPT
    : nh_(nh),
      data_mutex_(),
      last_data_(),
      new_data_ready_(false),
      is_simulated_time_(simulation),
      timer_(),
      timed_dt_(0),
      sim_dt_(0),
      stamped_dt_(0),
      message_count_(0),
      subscriber_(nh_.subscribe(topic_name, 100,
                                &StateController<Tp_>::Callback, this)) {
  timer_.Start();
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE StateController<Tp_>::~StateController() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE void StateController<Tp_>::Callback(const DataType &msg)
    ATLAS_NOEXCEPT {
  if (new_data_ready_) {
    ROS_WARN(
        "Buffer overflow in the state controller, a data has been skipped");
  }
  new_data_ready_ = true;
  ++message_count_;
  std::lock_guard<std::mutex> guard(data_mutex_);

  if (message_count_ > 1) {
    static const auto us = std::pow(10, -6);
    stamped_dt_ = ros::Time(msg.header.stamp).toSec() -
                  ros::Time(last_data_.header.stamp).toSec();
    timed_dt_ = timer_.MicroSeconds() * us;
    timer_.Reset();
  } else {
    stamped_dt_ = 0;
    timed_dt_ = 0;
    timer_.Reset();
  }
  last_data_ = msg;
}

//------------------------------------------------------------------------------
//
template <>
ATLAS_INLINE void StateController<std_msgs::Float64>::Callback(
    const DataType &msg) ATLAS_NOEXCEPT {
  // This is a temporary method for the barometer because the double64 message
  // does not have any header.
  // Anyway we don't use the dt of the baro...
  // TODO: Delete this method when we subscribe to FluidPressure messages
  new_data_ready_ = true;
  ++message_count_;
  std::lock_guard<std::mutex> guard(data_mutex_);
  timed_dt_ = timer_.MicroSeconds() * std::pow(10, -6);
  timer_.Reset();
  last_data_ = msg;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE Tp_ StateController<Tp_>::GetLastData() ATLAS_NOEXCEPT {
  new_data_ready_ = false;
  std::lock_guard<std::mutex> guard(data_mutex_);
  return last_data_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE bool StateController<Tp_>::IsNewDataReady() const ATLAS_NOEXCEPT {
  return new_data_ready_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE double StateController<Tp_>::GetDeltaTime() const ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(data_mutex_);
  if (is_simulated_time_) {
    return GetSimulatedDeltaTime();
  } else {
    return GetStampedDeltaTime();
  }
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE double StateController<Tp_>::GetTimedDeltaTime() const
    ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(data_mutex_);
  return timed_dt_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE double StateController<Tp_>::GetSimulatedDeltaTime() const
    ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(data_mutex_);
  return sim_dt_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE double StateController<Tp_>::GetStampedDeltaTime() const
    ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(data_mutex_);
  return stamped_dt_;
}

}  // namespace proc_navigation
