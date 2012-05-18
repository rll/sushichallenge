
import numpy as np
import scipy.interpolate as si

def shortest_path(ncost_nk,ecost_nkk):
    """
    ncost_nk: N x K
    ecost_nkk (N-1) x K x K
    """
    N,K = ncost_nk.shape
    cost_nk = np.empty((N,K),dtype='float')
    prev_nk = np.empty((N-1,K),dtype='int')
    cost_nk[0] = ncost_nk[0]
    for n in xrange(1,N):
        cost_kk = ecost_nkk[n-1] + cost_nk[n-1][:,None] + ncost_nk[n][None,:]
        cost_nk[n] = cost_kk.min(axis=0)
        prev_nk[n-1] = cost_kk.argmin(axis=0)

    cost = cost_nk[N-1].min()

    path = np.empty(N,dtype='int')    
    path[N-1] = cost_nk[N-1].argmin()
    for n in xrange(N-1,0,-1):
        path[n-1] = prev_nk[n-1,path[n]]
        
    return path,cost

def remove_duplicate_rows(mat):
    diffs = mat[1:] - mat[:-1]
    return mat[np.r_[True,(abs(diffs) >= 1e-6).any(axis=1)]]

def loglinspace(a,b,n):
    return np.exp(np.linspace(np.log(a),np.log(b),n))

def make_traj_with_limits(positions, vel_limits_j, acc_limits_j, smooth = False, plotting = False):
    positions, vel_limits_j, acc_limits_j = np.asarray(positions), np.asarray(vel_limits_j), np.asarray(acc_limits_j)
    
    positions = remove_duplicate_rows(positions)
    
    move_nj = positions[1:] - positions[:-1]
    mindur_n = (np.abs(move_nj) / vel_limits_j[None,:]).max(axis=1) # time according to velocity limit

    # maximum duration is set by assuming that you start out at zero velocity, accelerate until max velocity, coast, then decelerate to zero
    # this might give you a triangular or trapezoidal velocity profile
    # "triangle" velocity profile : dist = 2 * (1/2) * amax * (t/2)^2
    # "trapezoid" velocity profile: dist = t * vmax - (vmax / amax) * vmax
    maxdur_triangle_n = np.sqrt(4 * np.abs(move_nj) / acc_limits_j[None,:]).max(axis=1)
    maxdur_trapezoid_n = (np.abs(move_nj)/vel_limits_j[None,:] + (vel_limits_j / acc_limits_j)[None,:]).max(axis=1)
    maxdur_n = np.max([maxdur_triangle_n, maxdur_trapezoid_n],axis=0)+1
    
    
    K = 20
    N = len(mindur_n)
    
    ncost_nk = np.empty((N,K))
    ecost_nkk = np.empty((N,K,K))
    
    dur_nk = np.array([loglinspace(mindur,maxdur,K) for (mindur,maxdur) in zip(mindur_n, maxdur_n)])
    
    ncost_nk = dur_nk
    
    
    acc_njkk = move_nj[1:,:,None,None]/dur_nk[1:,None,None,:] - move_nj[:-1,:,None,None]/dur_nk[1:,None,:,None]
    ratio_nkk = np.abs(acc_njkk / acc_limits_j[None,:,None,None]).max(axis=1)
    ecost_nkk = 1e10 * (ratio_nkk > 1)
    
    path, cost = shortest_path(ncost_nk, ecost_nkk)
    print "cost: ", cost
    
    dur_n = [dur_nk[n,k] for (n,k) in enumerate(path)]
    times = np.cumsum(np.r_[0,dur_n])

    # use cubic spline to smooth it out
    if smooth: s = .01**2*(n+1)
    else: s = 0
    
    deg = min(3, len(positions) - 1)
    (tck, _) = si.splprep(positions.T, s=s, u=times, k=deg)
    smooth_positions = np.r_[si.splev(times,tck,der=0)].T
    smooth_velocities = np.r_[si.splev(times,tck,der=1)].T    
    return smooth_positions, smooth_velocities, times    
    #     
    if plotting:
        import matplotlib.pyplot as plt
        plt.figure(1)
        plt.plot(smooth_positions)
        plt.figure(2)
    
        plt.plot(smooth_velocities)
        plt.show()
    
    