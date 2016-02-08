/**
 * \file	extended_kalman_filter.h
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

#ifndef PROC_NAVIGATION_KALMAN_EXTENDED_KALMAN_FILTER_H_
#define PROC_NAVIGATION_KALMAN_EXTENDED_KALMAN_FILTER_H_

#include <memory>
#include <vector>
#include <lib_atlas/macros.h>
#include "proc_navigation/kalman/ekf_configuration.h"

namespace proc_navigation {

class ExtendedKalmanFilter : private EkfConfiguration {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ExtendedKalmanFilter>;
  using ConstPtr = std::shared_ptr<const ExtendedKalmanFilter>;
  using PtrList = std::vector<ExtendedKalmanFilter::Ptr>;
  using ConstPtrList = std::vector<ExtendedKalmanFilter::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ExtendedKalmanFilter(const EkfConfiguration &conf) ATLAS_NOEXCEPT;

  ~ExtendedKalmanFilter() ATLAS_NOEXCEPT;

  //==========================================================================
  // P U B L I C   M E T H O D S

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  //==========================================================================
  // P R I V A T E   M E M B E R S
};

}  // namespace proc_navigation

#endif  // PROC_NAVIGATION_KALMAN_EXTENDED_KALMAN_FILTER_H_
