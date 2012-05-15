#!/usr/bin/env python
import roslib
roslib.load_manifest("stack_plates")
import rospy
import sys
from geometry_msgs.msg import PointStamped, PoseStamped, QuaternionStamped, Quaternion
import tf
import tf.transformations as tft
from pr2_python import gripper
from numpy import pi
import pr2_python.arm_mover as pr2_am

#import copy
#from numpy import *
#import actionlib
#import time
#import thread
#import cv
#import yaml

#click twice, frist time on the closest edge point of plate
#second time on the center of plate

def callback2(x1,y1,z1,x2,y2,z2,frame_id, push_lr):

    print "blocking plate, point at:", x1, '/', y1, '/', z1,'/','//', 'pushing plate at', x2, '/', y2, '/', z2,'/',frame_id
    
    BRETT.lgrip.open()
    if push_lr == 'l':
        dy = 0.05
        g_yaw = -pi/2

        #blocking hand
        BRETT.larm.goto_cart_xyzrpy(x=x1,y=y1+0.20,z=z1,roll=pi/2,pitch=pi/6,yaw=-pi/2,block=True,target_frame=frame_id)
        BRETT.larm.goto_cart_xyzrpy(x=x1,y=y1-0.02,z=z1,roll=pi/2,pitch=pi/6,yaw=-pi/2,t=3,block=True,target_frame=frame_id)
        
        #pushing plate
        BRETT.rarm.goto_cart_xyzrpy(x=x2,y=y2,z=z2+0.20,     roll=pi/2,pitch=pi/2,yaw=0,    block=True,target_frame=frame_id)
        BRETT.rarm.goto_cart_xyzrpy(x=x2,y=y2,z=z2-0.01,     roll=pi/2,pitch=pi/2,yaw=0,t=3,block=True,target_frame=frame_id)
        BRETT.rarm.goto_cart_xyzrpy(x=x2,y=y2+0.05,z=z2-0.01,roll=pi/2,pitch=pi/2,yaw=0,t=3,block=True,target_frame=frame_id)

        #grasping plate
        BRETT.larm.goto_cart_xyzrpy(x=x1,y=y1+0.020,z=z1,roll=pi/2,pitch=pi/6,yaw=-pi/2,t=3,block=True,target_frame=frame_id)
        BRETT.lgrip.scoot_close_right()
        BRETT.rarm.goto_cart_xyzrpy(x=x2,y=y2+0.05,z=z2+0.20,roll=pi/2,pitch=pi/2,yaw=0,t=3,block=True,target_frame=frame_id)
        BRETT.larm.goto_cart_xyzrpy(x=x1,y=y1+0.020,z=z1+0.05,roll=pi/2,pitch=pi/12,yaw=-pi/2,t=3,block=True,target_frame=frame_id)
    else:
        dy = -0.05
        g_yaw = pi/2

def grasp_plate(bounding_box, tf_listener=None , larm_mover=None, rarm_mover=None, lgripper=None, rgripper=None, push_lr='l'):
    #grasps a single plate from a point cloud bounding box
    #TODO1: only works for circular plates
    #TODO2: grabs the left edge of the plate pusl_lr argument does nothing yet

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

    #transforms bounding box pose from unkown frame to torso_lift_link frame
    tll_pose = tf_listener.transformPoint('torso_lift_link', bounding_box.pose) 
    box_center = (tll_pose.position.x, tll_pose.position.y, tll.position.z)
    frame_id = bounding_box.header.frame_id
    box_size = (bounding_box.box_dims.x , bounding_box.box_dims.y , bounding_box.box_dims.z)

    print 'box at:', box_center , ' torso_lift_link // size of:', box_size
   
    edge_x = box_center                 #center of plate
    edge_y = box_center + box_size[1]/2 #left edge
    edge_z = box_center - box_size[2]/2 #table height

    #side of plate
    goal_side = xyzrpy_goal(edge_x, edge_y+0.03, edge_z, pi/2, pi/6, -pi/2, 'torso_lift_link')
    larm_mover.move_to_goal_directly(plate_side,5.0,None,False,4)

    #scoot in
    goal_side.pose.position.y = edge_y - 0.02
    larm_mover.move_to_goal_directly(goal_side,5.0,None,False,4)

    #shift left gripper right and close
    goal_shift = xyzrpy_goal(0,-0.045,0,0,0,'l_gripper_tool_frame')
    larm_mover.move_to_goal_directly(goal_shift,5.0,None,False,4)
    lgripper.close()

    #lift goal
    goal_side = xyzrpy_goal(edge_x, edge_y-0.02, edge_z+0.05, pi/2, pi/11, -pi/2, 'torso_lift_link')



def xyzrpy_goal(x,y,z,roll,pitch,yaw,frame_id):
    goal.pose.header.frame_id = frame_id
    goal = PoseStamped()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    quat_array =  tft.quaternion_from_euler(roll, pitch, yaw)
    quat = Quaternion()
    quat.x = quat_array[0]
    quat.y = quat_array[1]
    quat.z = quat_array[2]
    quat.w = quat_array[3]
    goal.pose.orientation = quat
    return goal

def main(args):
    rospy.init_node("test_pr2_unstack_plates")
    print "node initiated"
    global BRETT
    global state
    BRETT = PR2()
    state = 0
    BRETT.larm.goto_posture('up')
    BRETT.rarm.goto_posture('up')
    BRETT.lgrip.open()
    BRETT.rgrip.close()
    rospy.Subscriber("stereo_points_3d", PointStamped, callback1)
    print "READY FOR CLICKS"
    rospy.spin()
	

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
