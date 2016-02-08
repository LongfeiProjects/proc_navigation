/**
 * \file	object_mapper.cc
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

#include "proc_navigation/controllers/baro_controller.h"
#include "proc_navigation/controllers/dvl_controller.h"
#include "proc_navigation/controllers/imu_controller.h"
#include "proc_navigation/kalman/extended_kalman_filter.h"

namespace proc_navigation {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ExtendedKalmanFilter::ExtendedKalmanFilter(const EkfConfiguration &conf)
    ATLAS_NOEXCEPT : EkfConfiguration(conf) {}

//------------------------------------------------------------------------------
//
ExtendedKalmanFilter::~ExtendedKalmanFilter() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::OnSubjectNotify(atlas::Subject<> &subject)
ATLAS_NOEXCEPT {
 if(dynamic_cast<BaroController*>(&subject) != nullptr) {
   UpdateBaroData();
 } else if (dynamic_cast<DvlController*>(&subject) != nullptr) {
   UpdateDvlData();
 } else if (dynamic_cast<ImuController*>(&subject) != nullptr) {
   UpdateImuData();
 }
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::Initiate() {

}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::Run() {
  while(IsRunning()) {
      if(new_data_ready_) {
          std::lock_guard<std::mutex> guard(processing_mutex_);
      }
  }
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateImuData() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(processing_mutex_);
  // TODO Thibaut Mattio: Get the IMU data here
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateDvlData() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(processing_mutex_);
  // TODO Thibaut Mattio: Get the DVL data here
}

//------------------------------------------------------------------------------
//
void ExtendedKalmanFilter::UpdateBaroData() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(processing_mutex_);
  // TODO Thibaut Mattio: Get the Baro data here
}

//------------------------------------------------------------------------------
//
bool ExtendedKalmanFilter::IsNewDataReady() const ATLAS_NOEXCEPT {
  return new_data_ready_;
}

}  // namespace proc_navigation
