#!/usr/bin/env python

import roslib
roslib.load_manifest('rotating_grasper')
import rospy
import math
import tf
import sys
#import pr2_simple_motions
import actionlib
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from pr2_simple_motions_srvs.srv import *
from pr2_common_action_msgs.msg import *
from pr2_controllers_msgs.msg import *
from tf.transformations import quaternion_about_axis, quaternion_multiply

from rotating_grasper.msg import *
from rotating_grasper.srv import *

if __name__ == '__main__':
    rospy.init_node('fixed_grasp_sender')
    command = RotatingGrasp()
    command.header.stamp = rospy.Time.now()
    command.header.frame_id = 'base_footprint'
    command.center = Point()
    command.center.x = 0.673
    command.center.y = -0.444
    command.center.z = 0.829
    
    command.initial = Point()
    command.initial.x = 0.669
    command.initial.y = -0.322
    command.initial.z = 0.818
    
    command.rotation_rate = (2 * math.pi) / 30.0 #one rotation in 30 seconds
    
    print 'waiting for service'
    rospy.wait_for_service('rotating_grasper')
    server = rospy.ServiceProxy('rotating_grasper', RotatingGrasper)
    try:
        print 'calling...'
        response = server(command)
        if response.success:
            print 'success!'
        else:
            print 'failed :('
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e