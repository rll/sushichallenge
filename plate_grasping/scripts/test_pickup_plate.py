#!/usr/bin/env python
PKG="plate_grasping"
import roslib
roslib.load_manifest(PKG)
import rospy, rospkg, yaml
import sys, os, subprocess
from geometry_msgs.msg import PointStamped, PoseStamped, QuaternionStamped, Quaternion, Vector3
import tf
import tf.transformations as tft
from pr2_python import gripper
from numpy import pi
import math
import numpy as np
import pr2_python.arm_mover as pr2_am
import pr2_python.arm_planner as pr2_ap
import sensor_msgs
from tf.transformations import quaternion_about_axis, quaternion_multiply, quaternion_matrix
from collections import defaultdict
from pr2_python import transform_listener, pointclouds

from misc_msgs.msg import *

import geometry_msgs

from pr2_python import base

DIR = roslib.packages.get_pkg_dir(PKG, required=True) + "/config/"
stream = file(DIR+"config.yaml")
config = yaml.load(stream)

def make_fuerte_env():
    versionstr = sys.version[:3]
    return dict(
        ROS_MASTER_URI = os.environ["ROS_MASTER_URI"],
        PATH = "/opt/ros/fuerte/bin:%s"%os.environ["PATH"],
        ROS_VERSION = "fuerte",
        PYTHONPATH = "/opt/ros/fuerte/lib/python%s/dist-packages"%versionstr,
        ROS_PACKAGE_PATH = "/opt/ros/fuerte/share:/opt/ros/fuerte/stacks")


def make_tracker():    
    rp = rospkg.RosPack()
    pkg_path = rp.get_path("spinning_tabletop_detection")    
    p = subprocess.Popen(["%s/bin/test_tracker_ros"%pkg_path
                           ,"input_cloud:=/camera/rgb/points"
                           ,"--min_height=%s"%config["min_filter_height"]
                           ,"--max_height=%s"%config["max_filter_height"]
                           ,"--above_table_cutoff=%s"%config["above_table_cutoff"]                                                      
                           ], env = make_fuerte_env(), stdout = open('/dev/null','w'))
    return p

class WorldState(object):
    def __init__(self, find_box):
        rospy.loginfo("Waiting for find_cluster_bounding_box2 service")
        self.find_box = find_box 
        self.items = defaultdict(list)
        self.list_of_item_types = defaultdict(list) 
        self.pickup_next = None #PickupGoal
        self.free_arms = ["right_arm", "left_arm"]
        self.dining_setup = {"top":False, "left":False}
        self.object_in_hands = {} #{left: obj, right:obj}
        self.filters = None
        self.failed_to_pickup = None
        self.table_locations = ["table_top_edge",
                                "table_left_edge",
                                "table_right_edge"]
        self.round_robin_location = 0


class PlateGrasper:
    def __init__(self,base_mover):
        self.world = WorldState(None)
        self.base = base_mover
        
        self.cylinders = defaultdict(list) #list of (center, radius)
        self.num_detections = 0
        self.tracker = None

    def grasp_plate(self):
        if len(self.world.free_arms) == 2:
            lr_force = None
        else:
            lr_force = self.world.free_arms[0]
        listener = transform_listener.get_transform_listener()
        try:
            if self.tracker is None or self.tracker.poll() is not None:
                self.tracker = make_tracker()

            self.done = False
            print 'subscribing'
            rospy.Subscriber('/spinning_tabletop/cylinders',TrackedCylinders,self.handle_detection)
            sleep_time = 10
            
            print 'waiting for %d seconds' % sleep_time
            rospy.sleep(sleep_time)
            self.done = True

            key = self.cylinders.keys()[0]

            cylinder = self.cylinders[key][0]

            x = cylinder[0].pose.position.x
            y = cylinder[0].pose.position.y
            z = cylinder[0].pose.position.z

            try:
                self.base.move_manipulable_pose(x, y, z,
                                                try_hard = True, group="torso")
            except Exception as e:
                rospy.logerr("Got an error: %s", e)
                return "failure"

            return grasp_plate_from_cylinder(cylinder,listener,lr_force=lr_force)
        except Exception as e:
	        rospy.logerr("Got an error: %s", e)
	        return "failure"

    def handle_detection(self,detection):
        if self.done: return
        for i in range(len(detection.ids)):
            pt = PointStamped()
            pt.header = detection.header
            pt.point.x = detection.xs[i]
            pt.point.y = detection.ys[i]
            pt.point.z = detection.zs[i] - detection.hs[i]
            self.cylinders[detection.ids[i]].append((pt,detection.rs[i], detection.hs[i]))
            self.num_detections += 1
            self.done = True


brett = None

outward_angle = pi/4.
offset_init = 0.05
offset_final = 0.
offset_vertical = 0.


def grasp_plate_from_cylinder(cylinder, tf_listener=None , larm_mover=None, rarm_mover=None, lgripper=None, rgripper=None, lr_force =None):


    if tf_listener == None:
        tf_listener = tf.TransformListener()
        rospy.loginfo('created tf lisener...waiting 1 second')
        rospy.sleep(1)
        rospy.loginfo('done waiting')

    if larm_mover == None:
        larm_mover = pr2_am.ArmMover('left_arm')
    if rarm_mover == None:
        rarm_mover = pr2_am.ArmMover('right_arm')
    if lgripper == None:
        lgripper = gripper.Gripper('left_arm')
    if rgripper == None:
        rgripper = gripper.Gripper('right_arm')
    
    global brett
    if brett == None:
        brett = PR2()


    bottom_center = cylinder[0]
    x = cylinder[0].point.x
    y = cylinder[0].point.y
    z = cylinder[0].point.z
    r = cylinder[1]
    h = cylinder[2]
    
    tll_point = tf_listener.transformPose('torso_lift_link', bottom_center)
    
    if tll_point.point.y > 0:
        side = 'left'
        arm_mover = larm_mover
        gripper = lgripper
        angles = np.linspace(math.pi/2,math.pi,math.pi/20)
        arm = brett.larm
        gripper_tool_frame = "l_gripper_tool_frame"
    else:
        side = 'right'
        arm_mover = rarm_mover
        gripper = rgripper
        angles = np.linspace(math.pi,3.*math.pi/2.,math.pi/20)
        arm = brett.rarm
        gripper_tool_frame = "r_gripper_tool_frame"
    
    for angle in angles:
        q1 = quaternion_about_axis(math.pi/2,(0,1,0))
        q2 = quaternion_about_axis(math.pi/2+angle,(1,0,0))
        q3 = quaternion_about_axis(-outward_angle,(0,0,1))
        q12 = quaternion_multiply(q1,q2)
        q = quaternion_multiply(q12,q3)
        pointing_axis = quaternion_matrix(q)[:3,0]
        
        target = geometry_msgs.msg.PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = cylinder.header.frame_id
        target.pose.position.x = x + math.cos(angle) * (r + offset_final) # - pointing_axis[0]*.08
        target.pose.position.y = y - math.sin(angle) * (r + offset_final) # - pointing_axis[1]*.08
        target.pose.position.z = z + offset_vertical # - pointing_axis[2]*.08 + half_height + .01
        
        target.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        
        #TODO: check ik
        import conversions
        mat4 = conversions.trans_rot_to_hmat([target.pose.position.x, target.pose.position.y, target.pose.position.z],q)
        joint_positions = arm.ik(mat4, target.header.frame_id, gripper_tool_frame)
        ik_worked = (joint_positions is not None)
        
        if not ik_worked:
            continue
        
        for offset in np.linspace(offset_init,offset_final,0.01):
            target = geometry_msgs.msg.PoseStamped()
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = cylinder.header.frame_id
            target.pose.position.x = x + math.cos(angle) * (r + offset) # - pointing_axis[0]*.08
            target.pose.position.y = y - math.sin(angle) * (r + offset) # - pointing_axis[1]*.08
            target.pose.position.z = z + offset_vertical # - pointing_axis[2]*.08 + half_height + .01
            
            target.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
            
            try:
                arm_mover.move_to_goal_directly(target,5.0,None,False,4,collision_aware=False)
            except:
                break
            
            return grab_success(side[0])
    return False

def grab_success(lr):
    msg = rospy.wait_for_message('/joint_states',sensor_msgs.msg.JointState)
    gripper_joint_angle = msg.position[msg.name.index("%s_gripper_joint"%lr)]
    return (gripper_joint_angle > .0025) 


rospy.init_node("test_pickup_plate")
base_mover = base.Base()

import traceback
try:
    grasper = PlateGrasper(base_mover)
    grasper.grasp_plate()
finally:
    traceback.print_exc()
    grasper.tracker.terminate()
