"""
Common numpy imports and useful functions
"""
from __future__ import division
import numpy as np
np.set_printoptions(precision=3)

def interp2d(x,xp,yp):
    "Same as np.interp, but yp is 2d"
    yp = np.asarray(yp)
    assert yp.ndim == 2
    return np.array([np.interp(x,xp,col) for col in yp.T]).T
def normalize(x):
    return x / np.linalg.norm(x)
def normr(x):
    assert x.ndim == 2
    return x/norms(x,1)[:,None]
def normc(x):
    assert x.ndim == 2
    return x/np.norms(x,0)[None,:]
def norms(x,ax):
    return np.sqrt((x**2).sum(axis=ax))
def intround(x):
    return np.round(x).astype('int32')
def deriv(x):
    T = len(x)
    return np.interp2d(np.arange(T),np.arange(.5,T-.5),x[1:]-x[:-1])
def linspace2d(start,end,n):
    cols = [np.linspace(s,e,n) for (s,e) in zip(start,end)]
    return np.array(cols).T