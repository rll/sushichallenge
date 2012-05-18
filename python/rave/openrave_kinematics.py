import openravepy
import numpy as np

ROS_JOINT_NAMES = ['fl_caster_rotation_joint', 'fl_caster_l_wheel_joint',
       'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint',
       'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint',
       'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint',
       'bl_caster_r_wheel_joint', 'br_caster_rotation_joint',
       'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint',
       'torso_lift_joint', 'torso_lift_motor_screw_joint',
       'head_pan_joint', 'head_tilt_joint', 'laser_tilt_mount_joint',
       'r_upper_arm_roll_joint', 'r_shoulder_pan_joint',
       'r_shoulder_lift_joint', 'r_forearm_roll_joint',
       'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint',
       'r_gripper_joint', 'r_gripper_l_finger_joint',
       'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint',
       'r_gripper_l_finger_tip_joint', 'r_gripper_motor_screw_joint',
       'r_gripper_motor_slider_joint', 'l_upper_arm_roll_joint',
       'l_shoulder_pan_joint', 'l_shoulder_lift_joint',
       'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint',
       'l_wrist_roll_joint', 'l_gripper_joint', 'l_gripper_l_finger_joint',
       'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint',
       'l_gripper_l_finger_tip_joint', 'l_gripper_motor_screw_joint',
       'l_gripper_motor_slider_joint']

class RaveRobot(object):
    def __init__(self, robotmodel = 'robots/pr2-beta-sim.robot.xml'):
        self.env = openravepy.Environment()
        self.env.Load(robotmodel)
        self.robot = self.env.GetRobots()[0]
        self.ros2rave = np.array([self.robot.GetJointIndex(name) for name in ROS_JOINT_NAMES])
        self.goodrosinds = np.flatnonzero(self.ros2rave != -1)
        self.raveinds = self.ros2rave[self.goodrosinds]
    def set_joint_positions(self, rosvalues):
        ravevalues = rosvalues[self.goodrosinds]
        self.robot.SetJointValues(ravevalues[:20], self.raveinds[:20])
        self.robot.SetJointValues(ravevalues[20:], self.raveinds[20:])

        