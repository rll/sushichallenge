#!/usr/bin/env python
import roslib
roslib.load_manifest("stack_plates")
import rospy
import sys
from geometry_msgs.msg import PointStamped, PoseStamped, QuaternionStamped, Quaternion, Vector3
from object_manipulation_msgs.srv import FindClusterBoundingBox
import tf
import tf.transformations as tft
from pr2_python import gripper
from numpy import pi
import math
import numpy as np
import pr2_python.arm_mover as pr2_am
import pr2_python.arm_planner as pr2_ap
import pr2_python.trajectory_tools as tt
import sensor_msgs

global larm_mover
global rarm_mover
global lgripper
global rgripper
global rarm_planner
global larm_glanner
global tf_listener


def grasp_plate(bounding_box, tf_listener=None , larm_mover=None, rarm_mover=None, lgripper=None, rgripper=None, lr_force =None):
    #grasps a single plate from a point cloud bounding box
    #lr_force to only consider using 'l' or 'r' arm for grasping
    #bounding box

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
    tll_pose = tf_listener.transformPose('torso_lift_link', bounding_box.pose) 
    box_center = np.array([tll_pose.pose.position.x, tll_pose.pose.position.y, tll_pose.pose.position.z])
    frame_id = bounding_box.pose.header.frame_id
    box_size = np.array([bounding_box.box_dims.x , bounding_box.box_dims.y , bounding_box.box_dims.z])
    quat = tll_pose.pose.orientation
    box_quat = np.array([quat.x,quat.y,quat.z,quat.w])

    #pull arms back
    x = 0.3
    y = 0.65
    z = 0.35
    goal_lside = xyzrpy_goal(x, y, z, 0, 0, 0, 'torso_lift_link')
    larm_mover.move_to_goal_directly(goal_lside,5.0,None,False,4)
    goal_rside = xyzrpy_goal(x, -y, z, 0, 0, 0, 'torso_lift_link')
    rarm_mover.move_to_goal_directly(goal_rside,5.0,None,False,4)

    #infer grasp
    grasp_inference = infer_grasp(box_center, box_size, box_quat, tf_listener, lr_force)
    x = grasp_inference[0]
    y = grasp_inference[1]
    z = grasp_inference[2]
    angle = grasp_inference[3]
    dx = -np.cos(angle) * 0.1
    dy = np.sin(angle) * 0.1
    print 'calculated approach values:', (x,y,z,angle,dx,dy) 
    if angle < 0:
    #use left hand
        print 'using left hand'
        lgripper.open()
        goal_side = xyzrpy_goal(x+dx, y+dy, z+0.1, pi/2, pi/7, angle, 'torso_lift_link')  #outside of plate
        larm_mover.move_to_goal_directly(goal_side,5.0,None,False,4)
        goal_side.pose.position.z -= 0.1
        larm_mover.move_to_goal_directly(goal_side,5.0,None,False,4) 

        goal_shift = xyzrpy_goal(0.14,0,0,0,0,0,'l_gripper_tool_frame') #scoot into plate
        larm_mover.move_to_goal_directly(goal_shift,5.0,None,False,4)
        lgripper.close()

        goal_lift = xyzrpy_goal(x, y, z+0.1, pi/2, pi/10, angle, 'torso_lift_link')  #outside of plate
        larm_mover.move_to_goal_directly(goal_lift,5.0,None,False,4)
        return grab_success('l')
           
    else:
        print 'using right hand'
        rgripper.open()
        goal_side = xyzrpy_goal(x+dx, y+dy, z+0.1, pi/2, pi/7, angle, 'torso_lift_link')  #outside of plate
        rarm_mover.move_to_goal_directly(goal_side,5.0,None,False,4)
        goal_side.pose.position.z -= 0.1
        rarm_mover.move_to_goal_directly(goal_side,5.0,None,False,4) 

        goal_shift = xyzrpy_goal(0.14,0,0,0,0,0,'r_gripper_tool_frame') #scoot into plate
        rarm_mover.move_to_goal_directly(goal_shift,5.0,None,False,4)
        rgripper.close()
        
        goal_lift = xyzrpy_goal(x, y, z+0.1, pi/2, pi/10, angle, 'torso_lift_link')  #outside of plate
        rarm_mover.move_to_goal_directly(goal_lift,5.0,None,False,4)
        return grab_success('r')


def grab_success(lr):
    msg = rospy.wait_for_message('/joint_states',sensor_msgs.msg.JointState)
    gripper_joint_angle = msg.position[msg.name.index("%s_gripper_joint"%lr)]
    return (gripper_joint_angle > .0025) 

def xyzrpy_goal(x,y,z,roll,pitch,yaw,frame_id):
    #print "received xyzrpy to change into goal:", (x,y,z,roll,pitch,yaw,frame_id)

    goal = PoseStamped()
    goal.header.frame_id = frame_id
    quat_array =  tft.quaternion_from_euler(roll, pitch, yaw)
    quat = Quaternion()
    quat.x = quat_array[0]
    quat.y = quat_array[1]
    quat.z = quat_array[2]
    quat.w = quat_array[3]
    goal.pose.orientation = quat

    #transform girpper_tool_fram to wrist_roll_link
    rot_mat = np.matrix(tft.quaternion_matrix(quat_array))
    frame_shift = 0.18 * np.array(rot_mat[:,0])
    goal.pose.position.x = x - frame_shift[0][0]
    goal.pose.position.y = y - frame_shift[1][0]
    goal.pose.position.z = z - frame_shift[2][0]

    return goal

def infer_grasp(box_center, box_size, box_quat, tf_listener,lr_force ):
    #box_center is a numpy array of x,y,z
    #box_size is a numpy array of x,y,z
    #box_quat is a quaternion
    #frame_id is the frame the bounding box is in

    #get rotation matrix from pose
    rot_mat = np.matrix(tft.quaternion_matrix(box_quat))

    #vectors from center to edges of bounding box
    dx_p = np.matrix([[box_size[0]/2],[0],[0],[0]])
    dy_p = np.matrix([[0],[box_size[1]/2],[0],[0]])
    
    #4 grasp points relative to center of bounding box
    d_g1 = np.transpose(np.array(rot_mat*dx_p))
    d_g2 = -d_g1 
    d_g3 = np.transpose(np.array(rot_mat*dy_p))
    d_g4 = -d_g3 

    #grasp points in bounding_box frame
    print "d_g1 and box_center:", (d_g1[0][0:3], box_center)
    g1 = d_g1[0][0:3] + box_center #epsilon to prevent divide by 0 errors
    g2 = d_g2[0][0:3] + box_center
    g3 = d_g3[0][0:3] + box_center
    g4 = d_g4[0][0:3] + box_center
    
    g1_dist = np.sum(g1 * g1)
    g2_dist = np.sum(g2 * g2)
    g3_dist = np.sum(g3 * g3)
    g4_dist = np.sum(g4 * g4)
    g_dists = [g1_dist, g2_dist, g3_dist,g4_dist]
    
    #if hand is forced, use closest approach on the side of the hand
    if lr_force == 'l':
        #yaw is negative
        yaw_threshold = -1
    elif lr_force == 'r':
        #yaw is positive
        yaw_threshold = 1 
   
    #'''DEBUGGINiG CODE
    #'''
    #g1_yaw = np.arctan((box_center[1]-g1[1])/(box_center[0]-g1[0]+1e-10))
    #g2_yaw = np.arctan((box_center[1]-g2[1])/(box_center[0]-g2[0]+1e-10))
    #g3_yaw = np.arctan((box_center[1]-g3[1])/(box_center[0]-g3[0]+1e-10))
    #g4_yaw = np.arctan((box_center[1]-g4[1])/(box_center[0]-g4[0]+1e-10))

    #print ">>>>>>GRASP POINTS AND YAWS and distances>>>>>>" 
    #print(g1,g1_yaw,g1_dist)
    #print(g2,g2_yaw,g2_dist)
    #print(g3,g3_yaw,g3_dist)
    #print(g4,g4_yaw,g4_dist)

    if (lr_force =='l') or (lr_force =='r'):
	#choose from grasp points availible for only 1 hand
        grasp_arr = []
        g1_yaw = np.arctan((box_center[1]-g1[1])/(box_center[0]-g1[0]+1e-10))
        g2_yaw = np.arctan((box_center[1]-g2[1])/(box_center[0]-g2[0]+1e-10))
        g3_yaw = np.arctan((box_center[1]-g3[1])/(box_center[0]-g3[0]+1e-10))
        g4_yaw = np.arctan((box_center[1]-g4[1])/(box_center[0]-g4[0]+1e-10))
        g_yaws = [g1_yaw, g2_yaw,g3_yaw,g4_yaw]

        for i in range(4):
            if (g_yaws[i]*yaw_threshold) > 0:
                grasp_arr += [g_dists[i]]
        grasp_arr.sort()
        if grasp_arr[0] == g1_dist:
            grasp_point = (g1[0],g1[0])
        elif grasp_arr[0] == g2_dist:
            grasp_point = (g2[0], g2[1]) 
        elif grasp_arr[0] == g3_dist:
            grasp_point = (g3[0], g3[1]) 
        else:
            grasp_point = (g4[0], g4[1]) 

    else:
        #selecter second closest point for grasping
        grasp_arr = [g1_dist, g2_dist, g3_dist, g4_dist]
        grasp_arr.sort()
        if grasp_arr[1] == g1_dist:
            grasp_point = (g1[0], g1[1]) 
        elif grasp_arr[1] == g2_dist:
            grasp_point = (g2[0], g2[1]) 
        elif grasp_arr[1] == g3_dist:
            grasp_point = (g3[0], g3[1]) 
        else:
            grasp_point = (g4[0], g4[1]) 

    grasp_yaw = np.arctan((box_center[1]-grasp_point[1])/(box_center[0]-grasp_point[0]+1e-10))
    print "grasping point at:", grasp_point,'/', 'with grasp yaw of:', grasp_yaw
    return (grasp_point[0], grasp_point[1], box_center[2] - (box_size[2]/2), grasp_yaw)

def callback1(data):

    global larm_mover
    global rarm_mover
    global lgripper
    global rgripper
    global rarm_planner
    global larm_glanner
    global tf_listener

    x1 = data.point.x
    y1 = data.point.y
    z1 = data.point.z
    frame_id = data.header.frame_id

    box = FindClusterBoundingBox()

    box.pose = xyzrpy_goal(x1,y1,z1,0,0,pi/4,frame_id)

    vec = Vector3()
    vec.x = 0.265
    vec.y = 0.265
    vec.z = 0.02
    box.box_dims = vec

    #tll_pose = tf_listener.transformPose('torso_lift_link', box.pose) 
    #box_center = np.array([tll_pose.pose.position.x, tll_pose.pose.position.y, tll_pose.pose.position.z])
    #box_size = np.array([box.box_dims.x , box.box_dims.y , box.box_dims.z])
    #quat = tll_pose.pose.orientation
    #box_quat = np.array([quat.x,quat.y,quat.z,quat.w])
    #infer_grasp(box_center, box_size, box_quat, tf_listener)
    
    grasp_plate(box, tf_listener, larm_mover, rarm_mover, lgripper, rgripper)
def main(args):

    global larm_mover
    global rarm_mover
    global lgripper
    global rgripper
    global rarm_planner
    global larm_glanner
    global tf_listener

    rospy.init_node("test_pr2_unstack_plates")
    print "node initiated"
   

    tf_listener = tf.TransformListener()
    rospy.loginfo('created tf lisener...waiting 1 second')
    rospy.sleep(1)
    rospy.loginfo('done waiting')

    x = 0.3
    y = 0.65
    z = 0.35
    larm_mover = pr2_am.ArmMover('left_arm')
    rarm_mover = pr2_am.ArmMover('right_arm')
    lgripper = gripper.Gripper('left_arm')
    rgripper = gripper.Gripper('right_arm')
    '''
    goal_lside = xyzrpy_goal(x, y, z, 0, 0, 0, 'torso_lift_link')
    larm_mover.move_to_goal_directly(goal_lside,5.0,None,False,4)
    goal_rside = xyzrpy_goal(x, -y, z, 0, 0, 0, 'torso_lift_link')
    rarm_mover.move_to_goal_directly(goal_rside,5.0,None,False,4)
    '''

    rospy.Subscriber("stereo_points_3d", PointStamped, callback1)
    print "READY FOR CLICKS"
    rospy.spin()
	

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
