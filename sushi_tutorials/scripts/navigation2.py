#!/usr/bin/env python

import roslib
roslib.load_manifest('sushi_tutorials')
import rospy
import sys

from pr2_python import base

rospy.init_node('move_base_example')
x, y, z = -1.1, 0.2, 0.75
b = base.Base()
poses = b.get_base_poses(x, y, z,'right_arm')
rospy.loginfo("Got {0} base poses.".format(len(poses)))
#rospy.loginfo("Poses are: %s", poses)
