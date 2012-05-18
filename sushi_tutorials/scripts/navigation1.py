#!/usr/bin/env python
import roslib
roslib.load_manifest('sushi_tutorials')
import rospy

from pr2_python import base

rospy.init_node('move_base_example')
b = base.Base()
b.move_to(2, 0.066, -1.640)
