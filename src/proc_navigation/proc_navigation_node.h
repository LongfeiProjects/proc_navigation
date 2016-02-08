/**
 * \file	navigation_node.h
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

#ifndef PROC_NAVIGATION_PROC_NAVIGATION_NODE_H_
#define PROC_NAVIGATION_PROC_NAVIGATION_NODE_H_

#include <memory>
#include <vector>
#include <lib_atlas/macros.h>
#include <ros/ros.h>
#include <proc_navigation/kalman/ekf_configuration.h>
#include <proc_navigation/kalman/extended_kalman_filter.h>

namespace proc_navigation {

class ProcNavigationNode {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ProcNavigationNode>;
  using ConstPtr = std::shared_ptr<const ProcNavigationNode>;
  using PtrList = std::vector<ProcNavigationNode::Ptr>;
  using ConstPtrList = std::vector<ProcNavigationNode::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ProcNavigationNode(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  ~ProcNavigationNode() ATLAS_NOEXCEPT;

  //==========================================================================
  // P U B L I C   M E T H O D S

  void Spin() ATLAS_NOEXCEPT;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;

  EkfConfiguration ekf_conf_;
  ExtendedKalmanFilter ekf_;
};

}  // namespace proc_navigation

#endif  // PROC_NAVIGATION_PROC_NAVIGATION_NODE_H_
