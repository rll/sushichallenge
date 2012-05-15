#!/usr/bin/env python

import roslib; roslib.load_manifest('sushi_tutorials')
import rospy
from pr2_tasks.tasks import Tasks

def main():
    tasks = Tasks()
    tasks.move_arms_to_side()

rospy.init_node('move_arms_to_side_node')
main()
