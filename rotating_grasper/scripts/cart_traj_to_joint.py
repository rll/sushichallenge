from brett2.pr2 import PR2
import rospy
import numpy as np
import scipy.interpolate as si
from time import sleep, time
import conversions as conv


def get_velocities(positions, times):
    positions = np.atleast_2d(positions)
    n = len(positions)
    deg = min(3, n - 1)
    (tck, _) = si.splprep(positions.T,s = .01**2*(n+1), u=times, k=deg)
    #smooth_positions = np.r_[si.splev(times,tck,der=0)].T
    velocities = np.r_[si.splev(times,tck,der=1)].T    
    return velocities 

def follow_cart_traj_with_grip(translations, rotations, times, grip_time):
    brett.update_rave()
    jpos = [brett.larm.cart2joint(conv.trans_rot_to_hmat(trans, rot)) for (trans,rot) in zip(translations,rotations)]
    jpos = np.unwrap(jpos, axis=0)
    jvel = get_velocities(jpos, times)

    brett.wait = False
    t_total = times[-1];
    brett.follow_timed_joint_trajectory(jpos, times)
    sleep(grip_time)
    brett.rgrip.set_angle_goal(0)
    sleep(t_total - grip_time)
    
