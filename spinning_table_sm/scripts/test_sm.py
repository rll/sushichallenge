#!/usr/bin/env python

PKG = "spinning_table_sm"
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros

from numpy import *

from collections import defaultdict

from spinning_table_sm import spinning_table_states
from spinning_table_sm.sm import create_spinning_table_sm

import misc_msgs

if __name__ == '__main__':
    rospy.init_node("spinning_table_sm", anonymous=True)
    
    #tasks = Tasks()
    
    sm = create_spinning_table_sm()
    
    try:
        outcome = sm.execute()
        rospy.loginfo("Outcome: %s", outcome)
    except Exception:
        sm.call_termination_cbs(None,None)
