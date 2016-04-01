/**
 * \file	simulation_data_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/03/2015
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

#include <gtest/gtest.h>
#include <lib_atlas/maths/matrix.h>
#include <lib_atlas/pattern/runnable.h>
#include <lib_atlas/sys/timer.h>
#include <nav_msgs/Odometry.h>
#include <proc_navigation/kalman/extended_kalman_filter.h>
#include <proc_navigation/proc_navigation_node.h>

using BaroMessage = proc_navigation::ExtendedKalmanFilter::BaroMessage;
using DvlMessage = proc_navigation::ExtendedKalmanFilter::DvlMessage;
using ImuMessage = proc_navigation::ExtendedKalmanFilter::ImuMessage;
using MagMessage = proc_navigation::ExtendedKalmanFilter::MagMessage;

static auto nh = std::shared_ptr<ros::NodeHandle>{nullptr};

static auto dt_imu = 0.01;
static auto dt_mag = 0.01;
static auto dt_dvl = 0.2857142857142857;
static auto dt_baro = 0.072;

static auto imu_topic = "/simulation_data/imu";
static auto mag_topic = "/simulation_data/mag";
static auto dvl_topic = "/simulation_data/topic";
static auto baro_topic = "/simulation_data/baro";

template <class Tp_>
class DevicePublisher : public atlas::Runnable {
 public:
  explicit DevicePublisher(const std::string topic_name, double dt,
                           const Tp_ &data, double duration = 10) noexcept
      : topic_name_(topic_name),
        data_(data),
        pub_(nh->advertise<Tp_>(topic_name_, 100)),
        dt_(dt),
        duration_(duration) {}

  ~DevicePublisher() noexcept = default;

  /// This should be specialized depending on the type of the data.
  auto GenerateData() const noexcept -> Tp_ { return Tp_(); }

  virtual auto Run() -> void override {
    auto count = 0;
    auto loop_rate = ros::Rate{1 / dt_};

    while (ros::ok() && IsRunning() && count * dt_ < duration_) {
      pub_.publish(data_);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
  }

 private:
  std::string topic_name_;
  Tp_ data_;

  ros::Publisher pub_;

  double dt_;
  double duration_;
};

class OdometrySubscriber {
 public:
  explicit OdometrySubscriber(const std::string &topic_name) noexcept
      : odom_(),
        count_(0),
        mutex_(),
        topic_name_(topic_name),
        sub_(nh->subscribe(topic_name, 1, &OdometrySubscriber::Callback,
                           this)) {}

  auto Callback(const nav_msgs::Odometry &msg) noexcept -> void {
    std::lock_guard<std::mutex> guard(mutex_);
    odom_ = msg;
    ++count_;
  };

  auto GetOdometry() const noexcept -> nav_msgs::Odometry {
    std::lock_guard<std::mutex> guard(mutex_);
    return odom_;
  }

 private:
  nav_msgs::Odometry odom_;
  int count_;
  mutable std::mutex mutex_;
  std::string topic_name_;
  ros::Subscriber sub_;
};

auto CreateNode() -> void {
  proc_navigation::ProcNavigationNode ph(*nh);
  ph.Spin();
}

TEST(SimulationDataTest, all_devices_ideal) {
  std::thread nav_thread(CreateNode);

  auto dvl_msg = DvlMessage();
  dvl_msg.twist.twist.linear.x = 1;
  dvl_msg.twist.twist.linear.y = 0;
  dvl_msg.twist.twist.linear.z = 0;
  DevicePublisher<DvlMessage> dvl_dev(dvl_topic, dt_dvl, dvl_msg);

  auto baro_msg = BaroMessage();
  baro_msg.fluid_pressure = 112638;
  DevicePublisher<BaroMessage> baro_dev(baro_topic, dt_baro, baro_msg);

  auto mag_msg = MagMessage();
  mag_msg.magnetic_field.x = -0.036;
  mag_msg.magnetic_field.y = -0.033;
  mag_msg.magnetic_field.z = 0.0406;
  DevicePublisher<MagMessage> mag_dev(mag_topic, dt_mag, mag_msg);

  auto imu_msg = ImuMessage();
  imu_msg.angular_velocity.x = 0;
  imu_msg.angular_velocity.y = 0;
  imu_msg.angular_velocity.z = 0;
  imu_msg.linear_acceleration.x = 0;
  imu_msg.linear_acceleration.y = 0;
  imu_msg.linear_acceleration.z = 0;
  DevicePublisher<ImuMessage> imu_dev(imu_topic, dt_imu, imu_msg);

  OdometrySubscriber odom_sub("/proc_navigation/odom");

  atlas::SecTimer timer;
  timer.Start();
  while (timer.Seconds() < 10) {
    ros::spinOnce();
  }

  auto odom = odom_sub.GetOdometry();
  ros::shutdown();
  nav_thread.join();
}

TEST(SimulationDataTest, all_devices_noisy) {
  std::thread nav_thread(CreateNode);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> d(0, 1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "proc_navigation");
  nh = std::make_shared<ros::NodeHandle>("~");

  nh->setParam("/proc_navigation/simulation/active", true);
  nh->setParam("/proc_navigation/simulation/dt_imu", dt_imu);
  nh->setParam("/proc_navigation/simulation/dt_mag", dt_mag);
  nh->setParam("/proc_navigation/simulation/dt_dvl", dt_dvl);
  nh->setParam("/proc_navigation/simulation/dt_baro", dt_baro);

  nh->setParam("/proc_navigation/topics/baro", imu_topic);
  nh->setParam("/proc_navigation/topics/dvl", mag_topic);
  nh->setParam("/proc_navigation/topics/imu", dvl_topic);
  nh->setParam("/proc_navigation/topics/mag", baro_topic);

  return RUN_ALL_TESTS();
}
