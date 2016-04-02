#!/bin/sh
# Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id  period(milliseconds)
#
# Yaw is rotation about Z. Pitch is rotation about Y. Roll is rotation about Z.
# Use the right hand rule, thumb in the direction of the axis and the finger curled aroud the axis.
# The direction of the finger is the the direction "plus" of the rotation.

rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 3.1415926535898 ENU NED 100
