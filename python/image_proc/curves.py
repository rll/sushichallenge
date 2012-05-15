import numpy as np
import cv2
import scipy.interpolate as si
import math_utils as mu

def remove_duplicate_rows(mat):
    diffs = mat[1:] - mat[:-1]
    return mat[np.r_[True,(abs(diffs) >= 1e-5).any(axis=1)]]

def unif_resample(x,n,tol=1,deg=3):    
    x = np.atleast_2d(x)
    x = remove_duplicate_rows(x)
    dl = mu.norms(x[1:] - x[:-1],1)
    l = np.cumsum(np.r_[0,dl])
    (tck,_) = si.splprep(x.T,k=deg,s = tol**2*len(x),u=l)
    
    newu = np.linspace(0,l[-1],n)
    return np.array(si.splev(newu,tck)).T
