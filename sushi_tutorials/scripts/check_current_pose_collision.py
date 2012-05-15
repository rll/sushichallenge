#!/usr/bin/env python
import roslib
roslib.load_manifest('sushi_tutorials')
import rospy
import sys
import sensor_msgs.msg as sm
import geometry_msgs.msg as gm

from pr2_python import base
import tf.transformations as trans
from pr2_python.transform_listener import transform_pose_stamped
from geometry_msgs.msg import PoseStamped

rospy.init_node('check_pose_collision', anonymous=True)

zero_pose = PoseStamped()
zero_pose.header.frame_id = "/base_footprint"
zero_pose.pose.orientation.w = 1
mappose = transform_pose_stamped("/map", zero_pose)
b = base.Base()
rospy.loginfo("Checking position for Pose: %s", mappose)
res = b.check_base_pose(mappose)
if res == True:
    rospy.loginfo("Pose is collision free")
else:
    rospy.logwarn("Pose is in collision!")
