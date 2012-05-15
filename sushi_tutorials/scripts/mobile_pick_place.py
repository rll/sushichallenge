#!/usr/bin/env python
import roslib; roslib.load_manifest('sushi_tutorials')
import rospy
from pr2_tasks.tasks import Tasks
from geometry_msgs.msg import PoseStamped

arm_name = 'right_arm'

def pickup_test(tasks):
    rospy.loginfo('Going to pickup!')
    tasks.go_and_pickup(arm_name, (1.7, 2.0, 1.0))

def place_test(tasks):
    #define a place goal
    place_stamped = PoseStamped()
    place_stamped.header.frame_id = '/map'
    place_stamped.pose.position.x = -1.1
    place_stamped.pose.position.y = 0.2
    place_stamped.pose.position.z = 0.75
    place_stamped.pose.orientation.w = 1.0

    rospy.loginfo('Going to place!')
    tasks.go_and_place(arm_name, place_stamped)

def main():
    tasks = Tasks()
    pickup_test(tasks)
    place_test(tasks)


rospy.init_node('mobile_pick_place_node')
main()

