#!/usr/bin/env python
import roslib; roslib.load_manifest('sushi_tutorials')
import rospy
from pr2_python.world_interface import WorldInterface
from pr2_python.planning_scene_interface import get_planning_scene_interface

def main():
    wi = WorldInterface()
    wi.reset()
    psi = get_planning_scene_interface()
    psi.reset()
    rospy.sleep(2.0)

rospy.init_node('reset_everything_node')
main()
