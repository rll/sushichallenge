#!/usr/bin/env python
import roslib; roslib.load_manifest('sushi_tutorials')
import rospy
from pr2_tasks.tableware_detection import TablewareDetection
from geometry_msgs.msg import PointStamped

def main():
    det = TablewareDetection()
    head_point = PointStamped()
    head_point.header.frame_id = '/torso_lift_link'
    head_point.point.x = 0.6
    res = det.detect_objects(point_head_at=head_point)
    rospy.loginfo('Detected objects with the following labels and poses:')
    for pg in res.pickup_goals:
        rospy.loginfo('Label: '+str(pg.label)+', Pose:\n'+str(pg.object_pose_stamped))
    return

rospy.init_node('tableware_detection_tester_node')
main()
