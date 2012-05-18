import numpy as np
import scipy.interpolate as si
from numpy import pi

def smaller_ang(x):
    return (x + pi)%(2*pi) - pi
def closer_ang(x,a,dir=0):
    """                                                                        
    find angle y (==x mod 2*pi) that is close to a                             
    dir == 0: minimize absolute value of difference                            
    dir == 1: y > x                                                            
    dir == 2: y < x                                                            
    """
    if dir == 0:
        return a + smaller_ang(x-a)
    elif dir == 1:
        return a + (x-a)%(2*pi)
    elif dir == -1:
        return a + (x-a)%(2*pi) - 2*pi
def closer_joint_angles(pos,seed):
    print "pos",pos
    print "seed",seed
    result = np.array(pos)
    for i in [-1,-3]:
        result[i] = closer_ang(pos[i],seed[i],0)
    print "result",result
    return result


def get_velocities(positions, times, tol):
    positions = np.atleast_2d(positions)
    n = len(positions)
    deg = min(3, n - 1)
    (tck, _) = si.splprep(positions.T,s = tol**2*(n+1), u=times, k=deg)
    #smooth_positions = np.r_[si.splev(times,tck,der=0)].T
    velocities = np.r_[si.splev(times,tck,der=1)].T    
    return velocities
