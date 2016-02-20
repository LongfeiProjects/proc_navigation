/**
 * \file	state_controller.cc
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

#include "proc_navigation/controllers/state_controller.h"

namespace proc_navigation {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
StateController::StateController(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT {}

//------------------------------------------------------------------------------
//
StateController::~StateController() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
double StateController::GetTimeDelta() const ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(time_mutex_);
  return delta_t_;
}

//------------------------------------------------------------------------------
//
bool StateController::IsNewDataReady() const ATLAS_NOEXCEPT {
  return new_data_ready_;
}

//------------------------------------------------------------------------------
//
void StateController::ReceivedNewData() ATLAS_NOEXCEPT {
  new_data_ready_ = true;
  std::lock_guard<std::mutex> guard(time_mutex_);
  delta_t_ = timer_.Time();
  timer_.Reset();
  Notify();
}

}  // namespace proc_navigation
