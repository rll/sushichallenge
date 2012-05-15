import numpy as np
from numpy import inf, zeros, dot, pi,r_, sqrt, linspace
from numpy.linalg import norm, inv
import scipy.interpolate as si

def make_joint_trajectory_with_limits(positions, vel_limits, acc_limits):
    positions = np.asarray(positions)
    vel_limits = np.asarray(vel_limits)
    acc_limits = np.asarray(acc_limits)
    n_waypoints, n_joints = positions.shape

    # estimate the time, to pick a reasonable number of samples
    t_est = (abs(positions[1:] - positions[:-1]).sum(axis=0) / vel_limits).max()
    # samples is an integer multiple of n_waypoints, so that we hit all waypoints
    upsample_ratio = max(1, int(np.ceil(t_est * 10 / n_waypoints)))
    n_samples = n_waypoints * upsample_ratio

    # upsample and smooth a little bit
    k = min(3, n_waypoints - 1)     
    (tck, _) = si.splprep(positions.T, s = .001**2*n_waypoints, u=linspace(0,1,n_waypoints), k=k) # todo: is s total or per waypoint?
    sampled_positions = r_[si.splev(linspace(0,1,n_samples),tck)].T


    velocities = np.zeros((n_samples, n_joints))
    times = np.zeros(n_samples)
    
    for i in xrange(1,n_samples):
        
        dpos = (sampled_positions[i] - sampled_positions[i-1])
        # amount of time if we're at velocity speed limit:
        dt_vel = norm(dpos / vel_limits, inf)

        # search for minimal dt that satisfies velocity and acceleration limits
        f = lambda dt: (abs(dpos/dt - velocities[i-1])/dt < acc_limits).all()
        dt = line_search(f, dt_vel)
        
        times[i] = times[i-1]+dt
        velocities[i] = dpos/dt

    return sampled_positions, velocities, times
    
def line_search(f, x):
    if f(x): return x
    while True:
        x = 2*x
        if f(x): break
    good = x
    bad = x/2
    for i in xrange(10):
        x = sqrt(good*bad)
        if f(x): good = x
        else: bad = x
    return good
    
def make_joint_trajectory_with_duration(positions, total_time):
    raise NotImplementedError
  
