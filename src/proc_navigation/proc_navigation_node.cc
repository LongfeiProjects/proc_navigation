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
#include <nav_msgs/Odometry.h>
#include "proc_navigation/proc_navigation_node.h"

namespace proc_navigation {

//==============================================================================
// C / D T O R S   S E C T I O N

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
      ekf_(baro_, imu_, mag_, dvl_, nh_),
      odom_pub_(nh_->advertise<nav_msgs::Odometry>("odom", 100)) {}

//-----------------------------------------------------------------------------
//
ProcNavigationNode::~ProcNavigationNode() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::Spin() ATLAS_NOEXCEPT {
  while (!ros::isShuttingDown()) {
    while (nh_->ok()) {
      ros::spinOnce();
    }
  }
}

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::OnSubjectNotify(atlas::Subject<> &subject) ATLAS_NOEXCEPT {
  nav_msgs::Odometry odom;
  odom.twist.twist.linear.x = ekf_.GetStates().vel_n(0);
  odom.twist.twist.linear.y = ekf_.GetStates().vel_n(1);
  odom.twist.twist.linear.z = ekf_.GetStates().vel_n(2);

  odom.pose.pose.orientation.x = ekf_.GetStates().b.x();
  odom.pose.pose.orientation.y = ekf_.GetStates().b.y();
  odom.pose.pose.orientation.z = ekf_.GetStates().b.z();
  odom.pose.pose.orientation.w = ekf_.GetStates().b.w();

  odom.pose.pose.position.x = ekf_.GetStates().pos_n(0);
  odom.pose.pose.position.y = ekf_.GetStates().pos_n(1);
  odom.pose.pose.position.z = ekf_.GetStates().pos_n(2);

  odom_pub_.publish(odom);
}

}  // namespace proc_navigation
