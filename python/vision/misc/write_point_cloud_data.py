import roslib;
roslib.load_manifest("rospy")
import rospy
rospy.init_node("write clouds",disable_signals = True)

from brett2.ros_utils import pc2xyzrgb
import sensor_msgs.msg as sm
import numpy as np

while not rospy.is_shutdown():
    fname = raw_input("filename?: ")
    
    print "waiting for point cloud on input topic"
    msg = rospy.wait_for_message("cloud_in", sm.PointCloud2)
    print "ok"
    
    xyz,_ = pc2xyzrgb(msg)
    np.savetxt(fname, np.squeeze(xyz))
    

