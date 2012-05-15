#!/usr/bin/env python

import roslib
roslib.load_manifest('sushi_tutorials')
import rospy
import sys

from pr2_python import base

rospy.init_node('move_base_example')
x, y, z = -1.1, 0.2, 0.75
base = base.Base()
base.move_manipulable_pose(x, y, z)
