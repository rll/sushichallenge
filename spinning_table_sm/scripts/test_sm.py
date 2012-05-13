#!/usr/bin/env python

PKG = "spinning_table_sm"
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros

from numpy import *

from collections import defaultdict

from spinning_table_sm import spinning_table_states

import misc_msgs

from pr2_tasks.tasks import Tasks

def create_sm(tasks):
    sm = smach.StateMachine(outcomes=["success",
                                      "failure"])
    with sm:
        # smach.StateMachine.add("move_arm",
        #             spinning_table_states.MoveArmToSide(tasks),
        #             transitions = {"success":"detect",
        #                            "failure":"failure"
        #                           }
        #             )
                    
        smach.StateMachine.add("detect",
                    spinning_table_states.GatherDetections(),
                    transitions = {"success":"fit_circle",
                                   "failure":"failure"
                                  }
                    )
        
        smach.StateMachine.add("fit_circle", 
                spinning_table_states.FitCircle(),
                    transitions = {"success":"grasp",
                                   "failure":"failure"
                                  }
                )
        
        smach.StateMachine.add("grasp", 
                spinning_table_states.ExecuteGrasp(),
                    transitions = {"success":"success",
                                   "failure":"failure"
                                  }
                )
    return sm
    
if __name__ == '__main__':
    rospy.init_node("spinning_table_sm", anonymous=True)
    
    #tasks = Tasks()
    
    sm = create_sm(None)
    
    outcome = sm.execute()
    rospy.loginfo("Outcome: %s", outcome)
