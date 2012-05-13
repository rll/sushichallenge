import roslib;
roslib.load_manifest("pr2_python")
import pr2_python as pr2py
import numpy as np
import conversions as conv
import brett2.traj_utils as tu
import math_utils as mu

TrajectoryPoint = np.dtype([("xyz_l",float,3),
                            ("quat_l",float,4),
                            ("joint_l", float, 3),
                            ("grip_l",float),
                            ("xyz_r",float,3),
                            ("quat_r",float,4),
                            ("joint_r", float, 7),
                            ("grip_r",float),
                            ("base", float, 3)])

def make_joint_traj(xyzs, quats, arm, filter_options = 0):
    "do ik and then fill in the points where ik failed"


    n = len(xyzs)
    assert len(quats) == n

    joints = []
    inds = []

    for i in xrange(n):
        joint = arm.manip.FindIKSolution(conv.trans_rot_to_hmat(xyzs[i], quats[i]), filter_options)
        if joint is not None: 
            joints.append(joint)
            inds.append(ind)

    print "found ik soln for %i of %i points"%(len(inds), n)
    joints2 = math_utils.interp2d(np.arange(n), inds, joints)
    return joints2
    

def follow_arm_base_traj(pr2, traj):
    ljoints_raw = make_joint_trajectory(arr["xyz_l"], arr["quat_l"])
    rjoints_raw = make_joint_trajectory(arr["xyz_r"], arr["quat_r"])

    traj["joint_l"] = ljoints_raw
    traj["joint_r"] = rjoints_raw

    pr2.wait = False

    pr2.larm.follow_timed_joint_trajectory(lpos, lvel, times)
    pr2.rarm.follow_timed_joint_trajectory(rpos, rvel, times)
    pr2.lgrip.follow_timed_trajectory(lgrip, times)
    pr2.rgrip.follow_timed_trajectory(rgrip, times)
    pr2.base.follow_timed_trajectory(bpos, times)

