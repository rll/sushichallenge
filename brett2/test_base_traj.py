from brett2.pr2 import PR2
import conversions as conv
import numpy as np
import geometry_msgs.msg as gm
import trajectory_msgs.msg as tm
import traj_utils as tu
import scipy.interpolate as si
import rospy
from time import time,sleep
import roslib

        
if rospy.get_name() == "/unnamed":
    rospy.init_node("test_base_traj",anonymous=True, disable_signals=True)

xyas = np.r_[0,0,0][None,:] - np.linspace(0,1,10)[:,None]
ts = np.linspace(0,10,10)

pub = rospy.Publisher("base_traj_controller/command", tm.JointTrajectory)
brett = PR2()
xyacur = np.array(brett.base.get_pose())

jt = tm.JointTrajectory()
for i in xrange(10):
    
    jtp = tm.JointTrajectoryPoint()
    jtp.time_from_start = rospy.Duration(ts[i])
    jtp.positions = xyas[i]+xyacur
    jt.points.append(jtp)
    
pub.publish(jt)

