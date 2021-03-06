//
// Created by jeremie on 9/14/16.
//

#include "depth_meter_data.h"
#include "sonia_msgs/BarometerMsg.h"
namespace proc_navigation {

DepthMeterData::DepthMeterData() : depth_m_(false) {
}

void DepthMeterData::DepthMeterCallback(const sonia_msgs::BarometerMsg &msg) {
  // Depth from message is in m
  depth_m_ = (msg.depth/1000.0f) ;

  SetNewDataReady();
}
}