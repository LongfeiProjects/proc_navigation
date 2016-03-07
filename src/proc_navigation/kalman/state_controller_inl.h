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
    const ros::NodeHandle &nh, const std::string &topic_name) ATLAS_NOEXCEPT
    : nh_(nh),
      data_mutex_(),
      last_data_(),
      new_data_ready_(false),
      timer_(),
      dt_(),
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
  new_data_ready_ = true;
  std::lock_guard<std::mutex> guard(data_mutex_);
  dt_ = timer_.MicroSeconds() * std::pow(10, -6);
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
  return dt_;
}

}  // namespace proc_navigation
