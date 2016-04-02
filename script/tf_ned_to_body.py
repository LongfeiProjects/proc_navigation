#!/usr/bin/python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('proc_navigation')

import rospy
import tf
from nav_msgs.msg import Odometry


class NedToBody:
    def __init__(self):
        self.sub_odom = rospy.Subscriber("/proc_navigation/odom", Odometry,
                                         self.odom_callback)

        while not rospy.is_shutdown():
            continue

    @staticmethod
    def odom_callback(msg):
        br = tf.TransformBroadcaster()
        transform = tf.Transform()
        transform.setOrigin(tf.Vector3(msg.pose.pose.position.x,
                                       msg.pose.pose.position.y,
                                       msg.pose.pose.position.z))
        q = tf.Quaternion(msg.pose.pose.orientation.x,
                          msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z,
                          msg.pose.pose.orientation.w)
        transform.setRotation(q)
        br.sendTransformation(tf.StampedTransform(transform, rospy.Time.now(), "NED", "BODY"))


if __name__ == '__main__':
    rospy.init_node('tf_ned_to_body')
    try:
        quat_to_euler = NedToBody()
    except rospy.ROSInterruptException:
        pass
