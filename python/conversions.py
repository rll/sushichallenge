import numpy as np
import transformations
import roslib; roslib.load_manifest("geometry_msgs")
import geometry_msgs.msg as gm

def pose_to_trans_rot(pose):
    return (pose.position.x, pose.position.y, pose.position.z),\
           (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

def hmat_to_trans_rot(hmat): 
    ''' 
    Converts a 4x4 homogenous rigid transformation matrix to a translation and a 
    quaternion rotation. 
    ''' 
    scale, shear, angles, trans, persp = transformations.decompose_matrix(hmat) 
    rot = transformations.quaternion_from_euler(*angles) 
    return trans, rot 

def trans_rot_to_hmat(trans, rot): 
    ''' 
    Converts a rotation and translation to a homogeneous transform. 

    **Args:** 

        **trans (np.array):** Translation (x, y, z). 

        **rot (np.array):** Quaternion (x, y, z, w). 

    **Returns:** 
        H (np.array): 4x4 homogenous transform matrix. 
    ''' 
    H = transformations.quaternion_matrix(rot) 
    H[0:3, 3] = trans 
    return H


def xya_to_trans_rot(xya):
    x,y,a = xya
    return np.r_[x, y, 0], yaw_to_quat(theta)
def trans_rot_to_xya(trans, rot):
    x = trans[0]
    y = trans[1]
    a = quat_to_yaw(rot)
    return (x,y,a)

def quat_to_yaw(q):
    e = transformations.euler_from_quaternion(q)
    return e[2]
def yaw_to_quat(yaw):
    return transformations.quaternion_from_euler(0, 0, yaw)

def quat2mat(quat):
    return transformations.quaternion_matrix(quat)[:3, :3]
def mat2quat(mat33):
    mat44 = np.eye(4)
    mat44[:3,:3] = mat33
    return transformations.quaternion_from_matrix(mat44)

def rod2mat(r):
    theta = norm(rod)
    r = rod/theta
    rx,ry,rz = r
    self.mat = (
        np.cos(theta)*np.eye(3)
        + (1 - cos(theta))*np.outer(r,r)
        + sin(theta)*array([[0,-rz,ry],[rz,0,-rx],[-ry,rx,0]]))
    return self