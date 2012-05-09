#!/usr/bin/env python

import roslib
roslib.load_manifest('rotating_grasper')
import rospy
import math
import tf
import sys
import subprocess
#import pr2_simple_motions
import actionlib
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from pr2_simple_motions_srvs.srv import *
from pr2_common_action_msgs.msg import *
from pr2_controllers_msgs.msg import *
from tf.transformations import quaternion_about_axis, quaternion_multiply

from rotating_grasper.msg import *
from rotating_grasper.srv import *

from pr2_python.controller_manager_client import ControllerManagerClient
from pr2_python.arm_mover import ArmMover

def fix_angle(angle):
    while angle > 2*math.pi:
        angle = angle - 2*math.pi
    while angle < 0:
        angle = angle + 2*math.pi
    return angle

def handle_rotating_grasp(req):
    command = req.command
    print 'received command:'
    print command
    
    print 'opening gripper'
    grip_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
    grip_client.wait_for_server()
    
    grip_pos = 0.8
    effort = -1
    grip_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = grip_pos, max_effort = effort)))
    result = grip_client.wait_for_result()
    
    pub = rospy.Publisher('r_cart/command_pose', geometry_msgs.msg.PoseStamped)
    seq = 0
    
    offset_init = rospy.get_param('offset_init',0.15)
    offset = offset_init
    offset_final = rospy.get_param('offset_final',offset_init)
    start_drop_angle = rospy.get_param('start_drop_angle',math.pi/2)
    start_grip_angle = rospy.get_param('start_grip_angle',math.pi - math.pi/6)
    finish_lift_angle = rospy.get_param('finish_lift_angle',math.pi + math.pi/4)
    sleep_time = rospy.get_param('sleep_time',1/20.)
    
    x_diff = command.initial.x - command.center.x
    y_diff = command.initial.y - command.center.y
    radius = math.hypot(x_diff,y_diff)
    init_angle = math.atan2(y_diff,x_diff)
    rotation_rate = command.rotation_rate
    outward_angle = command.outward_angle
    
    print 'radius = %.3f, init angle = %.3fdeg' % (radius, (180*init_angle/math.pi))
    
    print 'going to initial pose'
    offset_vertical   = offset * math.cos(outward_angle)
    offset_horizontal = offset * math.sin(outward_angle)
    
    target = geometry_msgs.msg.PoseStamped()
    target.header.stamp = rospy.Time.now()
    target.header.frame_id = command.header.frame_id
    target.pose.position.x = command.center.x + math.cos(start_drop_angle) * (radius + offset_horizontal)
    target.pose.position.y = command.center.y - math.sin(start_drop_angle) * (radius + offset_horizontal)
    target.pose.position.z = command.center.z + offset_vertical
    q1 = quaternion_about_axis(math.pi/2,(0,1,0))
    q2 = quaternion_about_axis(math.pi/2+start_drop_angle,(1,0,0))
    q3 = quaternion_about_axis(-outward_angle,(0,0,1))
    q12 = quaternion_multiply(q1,q2)
    q = quaternion_multiply(q12,q3)
    #q = (0,0,0,1)
    target.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
    mover = ArmMover('right_arm')
    mover.move_to_goal(target,try_hard=False)
    
    #start controller
    print 'starting controller'
    #cmclient = ControllerManagerClient()
    #cmclient.switch_controllers(["l_cart"], ["l_arm_controller"])
    subprocess.check_call("rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller",shell=True)
    subprocess.check_call("rosrun pr2_controller_manager pr2_controller_manager start r_cart",shell=True)
    
    last_print = rospy.Time.now()
    
    timediff = 0
    has_been_past_perigee = False
    has_drop_started = False
    has_grip_started = False
    has_lift_started = False
    while True:
        now = rospy.Time.now() 
        timediff = (now - command.header.stamp).to_sec()
        
        #now_angle = math.fmod(init_angle + timediff * rotation_rate,math.pi)
        now_angle = fix_angle(init_angle + timediff * rotation_rate)
        if (now - last_print).to_sec() > 0.5:
            print 'current angle: %.3fdeg' % (180*now_angle/math.pi)
            last_print = now
        
        if now_angle > math.pi and not has_been_past_perigee:
            print 'passed perigee'
            has_been_past_perigee = True
        
        if has_been_past_perigee:
            if start_drop_angle <= now_angle and now_angle < start_grip_angle:
                if not has_drop_started:
                    print 'dropping'
                    has_drop_started = True
                offset = offset_init * (start_grip_angle - now_angle) / (start_grip_angle-start_drop_angle)
            elif has_drop_started and start_grip_angle <= now_angle and not has_grip_started:
                print 'starting grip'
                offset = 0
                
                #send gripper command
                grip_pos = 0
                effort = -1
                grip_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = grip_pos, max_effort = effort)))
                has_grip_started = True
            elif math.pi <= now_angle and has_grip_started:
                if not has_lift_started:
                    print 'lifting'
                    has_lift_started = True
                if finish_lift_angle <= now_angle:
                    #TODO: determine success
                    print 'Grasp finished'
                    return RotatingGrasperResponse(True)
                offset = offset_init * (now_angle - math.pi) / (finish_lift_angle - math.pi)
        
        if not has_drop_started:
            command_angle = start_drop_angle
        else:
            command_angle = now_angle
        
        if offset < 0:
            print 'offset negative!'
            offset = 0
        elif offset > offset_init:
            print 'offset too high!'
            offset = offset_init
        
        offset_vertical   = offset * math.cos(outward_angle)
        offset_horizontal = offset * math.sin(outward_angle)
        
        target = geometry_msgs.msg.PoseStamped()
        target.header.stamp = now
        target.header.frame_id = command.header.frame_id
        target.pose.position.x = command.center.x + math.cos(command_angle) * (radius + offset_horizontal)
        target.pose.position.y = command.center.y - math.sin(command_angle) * (radius + offset_horizontal)
        target.pose.position.z = command.center.z + offset_vertical
        
        
        q1 = quaternion_about_axis(math.pi/2,(0,1,0))
        q2 = quaternion_about_axis(math.pi/2+command_angle,(1,0,0))
        q3 = quaternion_about_axis(-outward_angle,(0,0,1))
        q12 = quaternion_multiply(q1,q2)
        q = quaternion_multiply(q12,q3)
        #q = (0,0,0,1)
        target.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        
        target.header.seq = seq
        
        pub.publish(target)
        seq = seq + 1
        
        rospy.sleep(sleep_time)
    return RotatingGrasperResponse(True)
    
def rotating_grasp_server():
    rospy.init_node('rotating_grasp_server')
    s = rospy.Service('rotating_grasper', RotatingGrasper, handle_rotating_grasp)
    
    grip_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
    grip_client.wait_for_server()
    
    print "Ready for grasping"
    rospy.spin()
        
if __name__ == '__main__':
    rotating_grasp_server()