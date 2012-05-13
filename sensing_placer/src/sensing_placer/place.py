#!/usr/bin/env python
import roslib; roslib.load_manifest('sensing_placer')
import rospy
from pr2_python.arm_mover import ArmMover
from pr2_python.arm_planner import ArmPlanner
from pr2_python.gripper import Gripper
from pr2_python.transform_listener import transform_pose, transform_pose_stamped
from geometry_msgs.msg import Pose
from pr2_python import base

#rospy.init_node('sensing_placer')

#rospy.spin()

class SensingPlacer:
    def __init__(self, arm_name):
        self.arm_name = arm_name
        self.mover = ArmMover(arm_name)
        self.planner = ArmPlanner(arm_name)
        self.gripper = Gripper(arm_name)
        self.base = base.Base()
        #self.gripper.close()

    def place(self, x, y, z):
        self.base.move_manipulable_pose(x, y, z, 
            try_hard = True, group=self.arm_name)
        world_pose = Pose()
        world_pose.position.x = x
        world_pose.position.y = y
        world_pose.position.z = z
        world_pose.orientation.x = 0.0
        world_pose.orientation.y = 0.0
        world_pose.orientation.z = 0.0
        world_pose.orientation.w = 1.0
       
        new_pose = transform_pose('base_footprint', 'map', world_pose)
        hand_pose = self.planner.get_hand_frame_pose()
        #print hand_pose
        #print new_pose
        hand_pose.pose.position.x = new_pose.position.x
        hand_pose.pose.position.y = new_pose.position.y
        hand_pose.pose.position.z = new_pose.position.z + 0.1

        self
        if self.arm_name == 'right_arm':
            joint_position = [-1.254, -0.325, -1.064, -1.525, 0.109, -1.185, 0.758]
        else:
            joint_position = [1.191, -0.295, 0.874, -1.610, 0.048, -1.069, -.988]
        joint_state = self.mover.get_joint_state()
        joint_state.position = joint_position
        self.mover.move_to_goal(joint_state)
        self.mover.move_to_goal_directly(hand_pose,collision_aware=False)
        #print "********"
        #print self.mover.get_exceptions()
        #print "********"

        still_in_free_space = True
        hand_pose = self.planner.get_hand_frame_pose()
        while still_in_free_space:
            goal_z = hand_pose.pose.position.z - 0.03
            hand_pose.pose.position.z = goal_z
            self.mover.move_to_goal_directly(hand_pose,collision_aware=False)
            hand_pose = self.planner.get_hand_frame_pose()
            if abs(hand_pose.pose.position.z - goal_z) > 0.01:
                still_in_free_space = False
            #print self.mover.reached_goal()
            #if not self.mover.reached_goal():
            #    still_in_free_space = False

        self.gripper.open()
        if self.arm_name == 'right_arm':
            gripper_name = 'r_gripper_palm_link'
        else:
            gripper_name = 'l_gripper_palm_link'
        gripper_pose = transform_pose_stamped(gripper_name, hand_pose)
        gripper_pose.pose.position.x -= 0.20
        hand_pose = transform_pose_stamped(hand_pose.header.frame_id, gripper_pose)
        self.mover.move_to_goal_directly(hand_pose, collision_aware=False)
        self.mover.move_to_goal(joint_state)
