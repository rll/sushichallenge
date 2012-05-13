#!/usr/bin/env python
"""
Author: Lorenzo Riano <lorenzo.riano@gmail.com>
"""
import roslib; 
roslib.load_manifest("sushi_sm")
import rospy
import smach
import smach_ros
from pr2_python import base
from sushi_sm import basic_states

def main():
    rospy.init_node("touring", anonymous=True)

    sm = smach.StateMachine(outcomes=["success",
                                      "failure"])
    base_mover = base.Base()
    poses = basic_states.poses
    
    nav_states = {}
    for name, pos in poses.iteritems():
        nav_states[name] = basic_states.NavigateTo(
        base_mover, pos[0], pos[1], pos[2])

    with sm:
        smach.StateMachine.add("start", nav_states["start"],
                transitions = {"success":"table_top_edge",
                               "failure":"failure"}
                )
        smach.StateMachine.add("table_top_edge", nav_states["table_top_edge"],
                transitions = {"success":"table_bottom_edge",
                               "failure":"failure"}
                )
        smach.StateMachine.add("table_bottom_edge", nav_states["table_bottom_edge"],
                transitions = {"success":"shelf",
                               "failure":"failure"}
                )
        smach.StateMachine.add("shelf", nav_states["shelf"],
                transitions = {"success":"other_table_top",
                               "failure":"failure"}
                )
        smach.StateMachine.add("other_table_top", nav_states["other_table_top"],
                transitions = {"success":"success",
                               "failure":"failure"}
                )
    outcome = sm.execute()
    rospy.loginfo("Outcome: %s", outcome)


if __name__ == "__main__":
    main()
        
