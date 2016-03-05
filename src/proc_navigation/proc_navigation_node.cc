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
ProcNavigationNode::ProcNavigationNode(const ros::NodeHandle &nh)
    ATLAS_NOEXCEPT
    : nh_(nh),
      ekf_conf_(nh_),
      baro_(std::make_shared<StateController<ExtendedKalmanFilter::BaroMessage>>(
              nh_, ekf_conf_.baro_topic)),
      imu_(std::make_shared<StateController<ExtendedKalmanFilter::ImuMessage>>(
          nh_, ekf_conf_.imu_topic)),
      mag_(std::make_shared<StateController<ExtendedKalmanFilter::MagMessage>>(
          nh_, ekf_conf_.mag_topic)),
      dvl_(std::make_shared<StateController<ExtendedKalmanFilter::DvlMessage>>(
          nh_, ekf_conf_.dvl_topic)),
      ekf_(baro_, imu_, mag_, dvl_, ekf_conf_),
      odom_pub_(nh_.advertise<nav_msgs::Odometry>("odom", 100)) {
  ekf_.Attach(*this);
}

//-----------------------------------------------------------------------------
//
ProcNavigationNode::~ProcNavigationNode() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::Spin() ATLAS_NOEXCEPT {
  while (nh_.ok()) {
    ros::spinOnce();
  }
}

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::OnSubjectNotify(atlas::Subject<> &subject) ATLAS_NOEXCEPT {
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";

  auto state = ekf_.GetStates();
  auto extra = ekf_.GetExtraStates();

  odom.twist.twist.linear.x = state.vel_n(0);
  odom.twist.twist.linear.y = state.vel_n(1);
  odom.twist.twist.linear.z = state.vel_n(2);

//  odom.pose.pose.orientation.x = state.b.x();
//  odom.pose.pose.orientation.y = state.b.y();
//  odom.pose.pose.orientation.z = state.b.z();
//  odom.pose.pose.orientation.w = state.b.w();

  // Completetely wrong but for tests
  odom.pose.pose.orientation.x = extra.euler(0);
  odom.pose.pose.orientation.y = extra.euler(1);
  odom.pose.pose.orientation.z = extra.euler(2);

  odom.pose.pose.position.x = state.pos_n(0);
  odom.pose.pose.position.y = state.pos_n(1);
  odom.pose.pose.position.z = state.pos_n(2);

  odom_pub_.publish(odom);
}

}  // namespace proc_navigation
