#!/usr/bin/env python
import roslib; roslib.load_manifest('sensing_placer')
import rospy
from sensing_placer.place import SensingPlacer
rospy.init_node('sensing_placer')
sp = SensingPlacer('right_arm')
sp.place(-2.15,.5,.98)
#sp.place(.5, -.19, .95)
#from pr2_python.transform_listener import transform_pose_stamped
#from pr2_python.arm_planner import ArmPlanner
#planner = ArmPlanner('right_arm')
#hand_pose = planner.get_hand_frame_pose()
#print hand_pose
#print transform_pose_stamped('map',hand_pose)
