#!/usr/bin/env python
import roslib
roslib.load_manifest('sushi_tutorials')
import rospy
import sys

from pr2_python import base

def main():
    rospy.init_node('move_to', anonymous=True)
    if len(sys.argv) != 4:
        rospy.logerr("Usage: %s x y th", sys.argv[0])
        rospy.loginfo("Args were: %s", sys.argv)
        sys.exit()
    b = base.Base()
    rospy.loginfo("Base pose: %s", b.get_current_pose())
    x = float(sys.argv[1].replace(',',''))
    y = float(sys.argv[2].replace(',',''))
    z = float(sys.argv[3].replace(',',''))
    pos = x,y,z
    rospy.loginfo("Sending command for pos: %s", pos)
    
    try:
        b.move_to(pos[0], pos[1], pos[2])
    except Exception as e:
        rospy.logerr("Something went wrong when moving, %s", e)
    rospy.loginfo("Done")

if __name__ == "__main__":
    main()
