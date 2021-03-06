//
// Created by jeremie on 9/14/16.
//

#ifndef PROC_NAVIGATION_DEPTHMETERDATA_HPP
#define PROC_NAVIGATION_DEPTHMETERDATA_HPP

#include <sensor_msgs/FluidPressure.h>
#include <proc_navigation/navigation_device.h>
#include <sonia_msgs/BarometerMsg.h>


namespace proc_navigation {

  class DepthMeterData : public NavigationDevice {
    public:
    DepthMeterData();

    void DepthMeterCallback(const sonia_msgs::BarometerMsg &msg);

    double GetDepth(){ return depth_m_;}

    private:
    double depth_m_;

  };
}

#endif //PROC_NAVIGATION_DEPTHMETERDATA_HPP
