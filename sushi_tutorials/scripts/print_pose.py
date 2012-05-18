#!/usr/bin/env python
import roslib
roslib.load_manifest('sushi_tutorials')
import rospy
import sys
import tf.transformations as trans
from pr2_python.transform_listener import transform_pose_stamped
from geometry_msgs.msg import PoseStamped

def _yaw(q):
    e = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return e[2]

rospy.init_node('get_current_pose', anonymous=True)
zero_pose = PoseStamped()
zero_pose.header.frame_id = "/base_footprint"
zero_pose.pose.orientation.w = 1
mappose = transform_pose_stamped("/map", zero_pose)
rospy.loginfo("Pose is %s", (mappose.pose.position.x, mappose.pose.position.y,
                             _yaw(mappose.pose.orientation) ))

