#!/usr/bin/env python
import roslib; roslib.load_manifest('sushi_tutorials')
import rospy
from pr2_tasks.tasks import Tasks
from pr2_tasks.pickplace import PickPlace
from pr2_tasks.tableware_detection import TablewareDetection
from pr2_tasks.pickplace_definitions import PlaceGoal
from geometry_msgs.msg import PointStamped
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
	pickplace = PickPlace()
	pg = det.pickup_goals[0]
	pg.arm_name = 'right_arm'
	pickplace.pickup(pg)

	# move the object 20 cm to the left of where we picked it up
	place_pose_stamped = copy.deepcopy(pg.object_pose_stamped)
	place_pose_stamped.pose.position.y += -0.0
	rospy.loginfo("Jenny frame: %s", place_pose_stamped)
	#place_pose_stamped.pose.position.x += 0.10
	place_goal = PlaceGoal(pg.arm_name, [place_pose_stamped],
		collision_support_surface_name = pg.collision_support_surface_name)

	# and put it back down
	pickplace.place(place_goal)
#end main

rospy.init_node('pickplace_tester_node')
main()
