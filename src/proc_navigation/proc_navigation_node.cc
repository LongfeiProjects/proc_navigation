/**
 * \file	navigation.cc
 * \author	Etienne Boudreault-Pilon <etienne.b.pilon@gmail.com>
 * \date	24/01/2016
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

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include "proc_navigation/proc_navigation_node.h"
#include "proc_navigation/kalman/extended_kalman_filter.h"

namespace proc_navigation {

//-----------------------------------------------------------------------------
//
ProcNavigationNode::ProcNavigationNode(const ros::NodeHandlePtr &nh)
    ATLAS_NOEXCEPT
    : nh_(nh),
      ekf_conf_(nh_),
      baro_(
          std::make_shared<StateController<ExtendedKalmanFilter::BaroMessage>>(
              nh_, ekf_conf_.baro_topic)),
      imu_(std::make_shared<StateController<ExtendedKalmanFilter::ImuMessage>>(
          nh_, ekf_conf_.imu_topic)),
      mag_(std::make_shared<StateController<ExtendedKalmanFilter::MagMessage>>(
          nh_, ekf_conf_.mag_topic)),
      dvl_(std::make_shared<StateController<ExtendedKalmanFilter::DvlMessage>>(
          nh_, ekf_conf_.dvl_topic)),
      ekf_(baro_, imu_, mag_, dvl_, nh_) {}

//-----------------------------------------------------------------------------
//
ProcNavigationNode::~ProcNavigationNode() ATLAS_NOEXCEPT {}

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::Spin() ATLAS_NOEXCEPT {
  while (!ros::isShuttingDown()) {
    while (nh_->ok()) {
      ros::spinOnce();
    }
  }
}

}  // namespace proc_navigation
