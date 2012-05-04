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

class RotatingGrasper:
    def __init__(self,rotation_rate):
        self.center_point = None
        self.init_point = None
        self.rotation_rate = rotation_rate
        
        self.radius = None
        self.init_angle = None
        
        self.pub = rospy.Publisher('r_cart/command_pose', geometry_msgs.msg.PoseStamped)
        self.seq = 0
        
        self.test = False
    
    def begin(self,listener):
        rospy.wait_for_service('rotating_grasp_server')
        self.server = rospy.ServiceProxy('rotating_grasp_server', RotatingGrasper)
        
        rospy.Subscriber('/stereo_points_3d',geometry_msgs.msg.PointStamped,self.handle_point)
        print 'ready for points (center then init)'
        rospy.spin()
    
    def handle_point(self,point):
        if self.center_point is None:
            print 'got center point'
            print point
            self.center_point = point
            return
        elif self.init_point is None:
            print 'got init point'
            print point
            self.init_point = point
            
            command = RotatingGrasp()
            command.header.stamp = rospy.Time.now()
            command.header.frame_id = 'base_footprint'
            command.center = Point()
            command.center.x = self.center_point.x
            command.center.y = self.center_point.y
            command.center.z = self.center_point.z
            
            command.initial = Point()
            command.initial.x = self.init_point.x
            command.initial.y = self.init_point.y
            command.initial.z = self.init_point.z
            
            command.rotation_rate = (2 * math.pi) / 30.0 #one rotation in 30 seconds
            
            try:
                response = self.server(command)
                if response.success:
                    print 'success!'
                else:
                    print 'failed :('
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    
        
if __name__ == '__main__':
    rospy.init_node('rotating_grasper')
    
    rotation_rate = (2 * math.pi) / 30.0 #one rotation in 30 seconds
    print 'using rotating rate %.3f' % rotation_rate
    grasper = RotatingGrasper(rotation_rate)
    
    listener = tf.TransformListener()
    
    grasper.begin(listener)