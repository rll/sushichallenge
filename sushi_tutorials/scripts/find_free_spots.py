#!/usr/bin/env python
import roslib; roslib.load_manifest('sushi_tutorials')
import rospy
from pr2_tasks.tasks import Tasks
from pr2_tasks.pickplace import PickPlace
from pr2_tasks.tableware_detection import TablewareDetection
from pr2_tasks.pickplace_definitions import PlaceGoal
from pr2_python.find_free_space import free_spots
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from object_manipulation_msgs.srv import FindClusterBoundingBox

import copy

def main():
	# find objects on the table    
    rospy.loginfo("Waiting for find_cluster_bounding_box2 service")
    find_box = rospy.ServiceProxy("/find_cluster_bounding_box", FindClusterBoundingBox)
    detector = TablewareDetection(table_thickness=0.1)
    det = detector.detect_objects(point_head_at=None)

    table = det.table
    table_dims = (table.x_max - table.x_min, table.y_max - table.y_min)
    rospy.loginfo("Table dims: %s", table_dims)
        
    objs = []
    for goal in det.pickup_goals:
        pointcloud = goal.target.cluster
        box = find_box(pointcloud)
        rospy.loginfo("----- Found label %s", goal.label)
        dims = box.box_dims
        pos = (box.pose.pose.position.x, box.pose.pose.position.y)
        obj = ( (pos[0] - dims.x/2 - table.x_min, pos[1] - dims.y/2 - table.y_min), (dims.x, dims.y))
        rospy.loginfo("Obj tuple: %s", obj)
        objs.append(obj)
    
    obj_dim = [(0.1, 0.10)]
    spots = free_spots(table_dims, obj_dim, 0.01, objs)
    rospy.loginfo("Spots: %s", spots)

        


rospy.init_node('detector_node')
main()
