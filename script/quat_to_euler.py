#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
PKG = 'proc_navigation'
import roslib

roslib.load_manifest(PKG)
import rospy
import tf

# ROS messages.
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sonia_msgs.msg import Eulers


class QuatToEuler:
    """ Class that subscribe to different ROS topics
        and output euler angles messages
    """
    def __init__(self):
        # Create subscribers and publishers.
        self.sub_imu = rospy.Subscriber("imu", Imu, self.imu_callback)
        self.sub_odom = rospy.Subscriber("/proc_navigation/odom", Odometry,
                                         self.odom_callback)
        self.pub_euler_odom = rospy.Publisher("/proc_navigation/euler_odom",
                                              Eulers)
        self.pub_euler_imu = rospy.Publisher("/proc_navigation/euler_imu",
                                             Eulers)

        # Main while loop.
        while not rospy.is_shutdown():
            continue

    # Odometry callback function.
    def odom_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        euler_msg = self.quat_to_euler_msg(msg, r, p, y)
        self.pub_euler_imu.publish(euler_msg)

    # IMU callback function.
    def imu_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z,
             msg.orientation.w])
        euler_msg = self.quat_to_euler_msg(msg, r, p, y)
        self.pub_euler_imu.publish(euler_msg)

    # Fill in Euler angle message.
    @staticmethod
    def quat_to_euler_msg(msg, r, p, y):
        euler_msg = Eulers()
        euler_msg.header.stamp = msg.header.stamp
        euler_msg.roll = r
        euler_msg.pitch = p
        euler_msg.yaw = y
        return euler_msg


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException:
        pass
