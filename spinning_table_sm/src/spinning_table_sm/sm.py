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

def create_spinning_table_sm():
    sm = smach.StateMachine(outcomes=["success",
                                      "failure"])
    
    with sm:
        smach.StateMachine.add("move_arm",
                    spinning_table_states.MoveArmToSide(),
                    transitions = {"success":"detect",
                                   "failure":"failure"
                                  }
                    )
                    
                    
        gd = spinning_table_states.GatherDetections()                    
        smach.StateMachine.add("detect",
                    gd,
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
                    transitions = {"success":"move_arm2",
                                   "failure":"failure",
                                   "missed":"move_arm"
                                  }
                )
        smach.StateMachine.add("move_arm2",
                    spinning_table_states.MoveArmToSide(),
                    transitions = {"success":"success",
                                   "failure":"failure"
                                  }
                    )        
    sm.register_termination_cb(gd.kill_tracker)
    return sm
    