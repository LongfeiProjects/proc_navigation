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

#include <typeinfo>
#include "proc_navigation/kalman/state_controller.h"

namespace proc_navigation {

template <class Tp_>
const uint32_t StateController<Tp_>::BUFFER_SIZE = 250;

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
      data_buffer_(),
      new_data_ready_(false),
      is_simulated_time_(simulation),
      first_stamped_t_(0),
      last_stamped_t_(0),
      current_stamped_t_(0),
      stamped_dt_(0),
      sim_dt_(sim_dt),
      message_count_(0),
      subscriber_(nh_.subscribe(topic_name, 100,
                                &StateController<Tp_>::Callback, this)) {}

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
  std::lock_guard<std::mutex> guard(data_mutex_);

  new_data_ready_ = true;
  ++message_count_;

  if (message_count_ == 1) {
    first_stamped_t_ = ros::Time(msg->header.stamp).toSec();
  }
  last_stamped_t_ = ros::Time(msg->header.stamp).toSec();

  if (data_buffer_.size() >= BUFFER_SIZE) {
    ROS_WARN_STREAM("Buffer overflow in the state controller of "
                    << typeid(msg).name() << ", a data has been skipped");
    data_buffer_.pop();
  }

  data_buffer_.push(msg);
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE typename StateController<Tp_>::DataType
StateController<Tp_>::GetLastData() {
  std::lock_guard<std::mutex> guard(data_mutex_);

  if (new_data_ready_) {
    if (data_buffer_.size() < 1) {
      throw std::logic_error(
          "The buffer is empty when it is supposed to contain data.");
    }

    current_data_ = data_buffer_.front();
    data_buffer_.pop();

    // If it is the first message, we want to have a null dt, thus, take the
    // last time at the time of the current data (difference will be 0).
    if (message_count_ == 1) {
      current_stamped_t_ = ros::Time(current_data_->header.stamp).toSec();
    }

    stamped_dt_ =
        ros::Time(current_data_->header.stamp).toSec() - current_stamped_t_;

    current_stamped_t_ = ros::Time(current_data_->header.stamp).toSec();

    new_data_ready_ = data_buffer_.size() > 0;
    return current_data_;
  } else {
    return current_data_;
  }
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE typename StateController<Tp_>::DataType
StateController<Tp_>::GetLastDataIfDtIn(double dt) {
  data_mutex_.lock();

  auto t_next = GetTimeForNext();
  if (t_next <= dt) {
    data_mutex_.unlock();
    return GetLastData();
  } else {
    data_mutex_.unlock();
    return nullptr;
  }
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
ATLAS_INLINE uint64_t
StateController<Tp_>::GetMessageCount() const ATLAS_NOEXCEPT {
  return message_count_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE double StateController<Tp_>::GetDeltaTime() const ATLAS_NOEXCEPT {
  if (is_simulated_time_) {
    return GetSimulatedDeltaTime();
  } else {
    return GetStampedDeltaTime();
  }
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE double StateController<Tp_>::GetTimeForLast() const
    ATLAS_NOEXCEPT {
  if (is_simulated_time_) {
    return (message_count_ - 1) * sim_dt_;
  } else {
    return last_stamped_t_ - first_stamped_t_;
  }
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE double StateController<Tp_>::GetTimeForCurrent() const
    ATLAS_NOEXCEPT {
  if (is_simulated_time_) {
    // Note that if there was a buffer overflow, this calculus is wrong...
    // Anyway, it is in sumalation mode and if there is a buffer overflow,
    // a problem must be solved elsewhere.
    return (message_count_ - data_buffer_.size()) * sim_dt_;
  } else {
    return current_stamped_t_ - first_stamped_t_;
  }
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE double StateController<Tp_>::GetTimeForNext() const
    ATLAS_NOEXCEPT {
  if (is_simulated_time_) {
    // Note that if there was a buffer overflow, this calculus is wrong...
    // Anyway, it is in sumalation mode and if there is a buffer overflow,
    // a problem must be solved elsewhere.
    return (message_count_ - data_buffer_.size() + 1) * sim_dt_;
  } else {
    auto data = data_buffer_.front();
    auto next_stamp = ros::Time(data->header.stamp).toSec();
    return next_stamp - first_stamped_t_;
  }
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
