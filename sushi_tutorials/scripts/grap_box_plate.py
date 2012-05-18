#!/usr/bin/env python
import roslib; roslib.load_manifest('sushi_tutorials')
import rospy
from pr2_tasks.tasks import Tasks
from pr2_tasks.pickplace import PickPlace
from pr2_tasks.tableware_detection import TablewareDetection
from pr2_tasks.pickplace_definitions import PlaceGoal
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from object_manipulation_msgs.srv import FindClusterBoundingBox
import grab_plate

import copy

def main():
	# find objects on the table    
    rospy.loginfo("Waiting for find_cluster_bounding_box2 service")
    find_box = rospy.ServiceProxy("/find_cluster_bounding_box", FindClusterBoundingBox)
    detector = TablewareDetection(table_thickness=0.1)
    det = detector.detect_objects(point_head_at=None)

    if not det.pickup_goals:
        rospy.loginfo('Nothing to pick up!')
        return
        
    for goal in det.pickup_goals:
        pointcloud = goal.target.cluster
        box = find_box(pointcloud)
        rospy.loginfo("----- Found label %s", goal.label)
        dims = box.box_dims
        rospy.loginfo("Bounding box dims: %.2f, %.2f, %.2f", dims.x, dims.y, dims.z)
        rospy.loginfo("Bounding box dims:%s", box)
        area = dims.x * dims.y * dims.z
        squareness = min(dims.x, dims.y) / max(dims.x, dims.y)
        rospy.loginfo("Area: %f", area)
        rospy.loginfo("Squareness: %f", squareness)
	if (goal.label == 'plate' or (dims.z <= 0.02)) and squareness > 0.5:
		grab_plate.grasp_plate(box) 


rospy.init_node('detector_node')
main()
