#!/usr/bin/env python

import roslib
roslib.load_manifest('rotating_grasper')
import rospy
import math
import tf
import sys
#import pr2_simple_motions
import actionlib
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from pr2_simple_motions_srvs.srv import *
from pr2_common_action_msgs.msg import *
from pr2_controllers_msgs.msg import *
from tf.transformations import quaternion_about_axis, quaternion_multiply

from rotating_grasper.msg import *
from rotating_grasper.srv import *

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
    
    offset_init = 0.15
    offset = offset_init
    start_drop_angle = 0
    start_grip_angle = math.pi - math.pi/6
    finish_lift_angle = 3*math.pi/2
    
    x_diff = command.initial.x - command.center.x
    y_diff = command.initial.y - command.center.y
    radius = math.hypot(x_diff,y_diff)
    init_angle = math.atan2(y_diff,x_diff)
    rotation_rate = command.rotation_rate
    print 'radius = %.3f, init angle = %.3fdeg' % (radius, (180*init_angle/math.pi))
    
    timediff = 0
    has_been_past_perigee = False
    has_drop_started = False
    has_grip_started = False
    has_lift_started = False
    while True:
        now = rospy.Time.now() #- rospy.Duration.from_sec(0.5)
        timediff = (now - command.header.stamp).to_sec()
        
        #now_angle = math.fmod(init_angle + timediff * rotation_rate,math.pi)
        now_angle = fix_angle(init_angle + timediff * rotation_rate)
        print 'current angle: %.3fdeg' % (180*now_angle/math.pi)
        
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
                #TODO send gripper command
                grip_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
                grip_client.wait_for_server()
                
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
                    return RotatingGrasperResponse(True)
                offset = offset_init * (now_angle - math.pi) / (finish_lift_angle - math.pi)
        
        if offset < 0:
            print 'offset negative!'
            offset = 0
        elif offset > offset_init:
            print 'offset too high!'
            offset = offset_init
        
        target = geometry_msgs.msg.PoseStamped()
        target.header.stamp = now
        target.header.frame_id = command.header.frame_id
        target.pose.position.x = command.initial.x + math.cos(now_angle)*radius
        target.pose.position.y = command.initial.y - math.sin(now_angle)*radius
        target.pose.position.z = command.initial.z + offset
        
        
        q1 = quaternion_about_axis(math.pi/2,(0,1,0))
        q2 = quaternion_about_axis(math.pi/2+now_angle,(1,0,0))
        q = quaternion_multiply(q1,q2)
        #q = (0,0,0,1)
        target.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        
        target.header.seq = seq
        
        pub.publish(target)
        seq = seq + 1
        
        rospy.sleep(0.5)
    return RotatingGrasperResponse(True)
    
def rotating_grasp_server():
    rospy.init_node('rotating_grasp_server')
    s = rospy.Service('rotating_grasper', RotatingGrasper, handle_rotating_grasp)
    print "Ready for grasping"
    rospy.spin()
        
if __name__ == '__main__':
    rotating_grasp_server()