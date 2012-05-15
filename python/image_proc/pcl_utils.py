import numpy as np

CX = 320-.5;
CY = 240-.5;
F = 525;

CAMERA_MATRIX = np.array([[F, 0, CX], [0, F, CY], [0, 0, 1]])

def xyz2uv(xyz): 
 # http://www.pcl-users.org/Using-Kinect-with-PCL-How-to-project-a-3D-point-x-y-z-to-the-depth-rgb-image-and-how-to-unproject-a--td3164499.html
  xyz = np.atleast_2d(xyz)
  x,y,z = xyz.T

  v = F*(x / z) + CX;
  u = F*(y / z) + CY;
  return np.c_[u,v].round().astype('int32')

def uv2xyz(uv,z):
  uv = np.atleast_2d(xyz)
  u,v = uv.T

  
  x = (u - CX)*(z/F)
  y = (u - CX)*(z/F)
  
  return np.array([x,y,z]).T

def to3d(uvs,xyz):
  us,vs = np.atleast_2d(uvs).T
  xyzs = xyz[us,vs]
  xyzs_good = xyzs[np.isfinite(xyzs).all(axis=1)]
  return xyzs_good