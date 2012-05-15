#!/usr/bin/env python
import roslib; roslib.load_manifest('sushi_tutorials')
import rospy
from pr2_tasks.tasks import Tasks
from pr2_tasks.pickplace import PickPlace
from pr2_tasks.tableware_detection import TablewareDetection
from pr2_tasks.pickplace_definitions import PlaceGoal
from pr2_python.gripper import Gripper
from geometry_msgs.msg import PointStamped
import sys
import copy

def main():
	# find objects on the table
	head_point = PointStamped()
	head_point.header.frame_id = '/torso_lift_link'
	head_point.point.x = 0.5
        head_point = None
	detector = TablewareDetection(table_thickness=0.1)
	det = detector.detect_objects(point_head_at=head_point)
	if not det.pickup_goals:
		rospy.loginfo('Nothing to pick up!')
		return

	# pick up the first object that we found
	if len(sys.argv) > 1:
		arm = sys.argv[1]
	else:
		arm = "right_arm"
	rospy.loginfo("Trying arm: %s", arm)
	pickplace = PickPlace()
	pg = det.pickup_goals[0]
	pg.arm_name = arm
	try:
		pickplace.pickup(pg)
	except Exception as e:
		rospy.logerr("No grasping, err: %s", e)
		return
	
	gripper = Gripper(arm)
	gripper.open()
	

#end main

rospy.init_node('pickplace_tester_node')
main()
