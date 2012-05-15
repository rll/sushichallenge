
import numpy as np

import roslib
roslib.load_manifest("rospy")
import rospy
roslib.load_manifest("sensor_msgs") 
import sensor_msgs.msg as sm
roslib.load_manifest("visualization_msgs")
import visualization_msgs.msg as vm
roslib.load_manifest("geometry_msgs")
import geometry_msgs.msg as gm
roslib.load_manifest("std_msgs")
import std_msgs.msg as stdm
from visualization_msgs.msg import Marker



def pc2xyzrgb(pc):
    arr = np.fromstring(pc.data,dtype='float32').reshape(pc.height,pc.width,8)
    xyz = arr[:,:,0:3]
    
    rgb0 = np.ndarray(buffer=arr[:,:,4].copy(),shape=(pc.height, pc.width,4),dtype='uint8')
    rgb = rgb0[:,:,0:3]
    
    return xyz,rgb
def xyzrgb2pc(xyz,bgr,frame_id = '/camera_rgb_optical_frame'):
    height= xyz.shape[0]
    width = xyz.shape[1]
    assert bgr.shape[0] == height
    assert bgr.shape[1] == width
    
    arr = np.empty((height,width,8),dtype='float32')
    arr[:,:,0:3] = xyz
    bgr1 = np.empty((height,width,4),dtype='uint8')
    bgr1[:,:,0:3] = bgr    
    arr[:,:,4] = bgr1.view(dtype='float32').reshape(height, width)
    data = arr.tostring()
    msg = sm.PointCloud2()
    msg.data = data
    msg.header.frame_id = frame_id
    msg.fields = [sm.PointField(name='x',offset=0,datatype=7,count=1),
                  sm.PointField(name='y',offset=4,datatype=7,count=1),
                  sm.PointField(name='z',offset=8,datatype=7,count=1),
                  sm.PointField(name='rgb',offset=16,datatype=7,count=1)]
    msg.is_dense = False
    msg.width=width
    msg.height=height
    msg.header.stamp = rospy.Time.now()
    msg.point_step = 32
    msg.row_step = 32 * width
    msg.is_bigendian = False
    
    return msg

def clone_msg(msg):
    sio = StringIO()
    msg.serialize(sio)
    newmsg = type(msg)()
    newmsg.deserialize(sio.getvalue())
    return newmsg

def msg2arr(msg):
    """convert a msg to an array of floats
    don't use on a stamped msg"""
    s = StringIO()
    msg.serialize_numpy(s,np)
    return np.fromstring(s.getvalue())

def arr2msg(arr,msg_type):
    """convert array of floats to msg
    don't use on a stamped msg type"""
    msg = msg_type()
    msg.deserialize_numpy(np.asarray(arr).tostring(),np)
    return msg

def point_stamed_to_pose_stamped(pts,orientation=[0,0,0,1]):
    """convert pointstamped to posestamped"""
    ps = gm.PoseStamped()
    ps.pose.position = pts.point
    ps.pose.orientation = orientation
    ps.header.frame_id = pts.header.frame_id
    return ps

def pose_to_pose_stamped(pose,frame_id = 'base_link'):
    ps = gm.PoseStamped()
    ps.pose = pose
    ps.header.frame_id = frame_id
    ps.header.stamp = rospy.Time.now()
    return ps

def pvec_to_pose_stamped(pvec,frame_id = 'base_link'):
    pose = arr2msg(pvec,gm.Pose)
    return pose2ps(pose,frame_id)

class TopicListener(object):
    """
    stores last message of the topic
    """
    last_msg = None
    def __init__(self,topic_name,msg_type):
        self.sub = rospy.Subscriber(topic_name,msg_type,self.callback)        
        
        rospy.loginfo('waiting for the first message: %s'%topic_name)
        while self.last_msg is None:
            time.sleep(.1)
        rospy.loginfo('ok: %s'%topic_name)        
    def callback(self,msg):
        self.last_msg = msg

def wait_for_service_verbose(service,timeout=None):
    rospy.loginfo('waiting for service: %s'%service)
    rospy.wait_for_service(service,timeout=timeout)
    rospy.loginfo('ok: %s'%service)
    
def wait_for_message_verbose(topic,msgtype,timeout=None):
    rospy.loginfo('waiting for message: %s'%topic)
    msg = rospy.wait_for_message(topic,msgtype,timeout=timeout)
    rospy.loginfo('ok: %s'%topic)
    return msg

def call_service(service,srv_class,req=None):
    wait_for_service_verbose(service)
    client = rospy.ServiceProxy(service,srv_class)
    if req is None: return client()
    else: return client(req)

def image2numpy(image):
    if image.encoding == 'rgb8':
        return np.fromstring(image.data,dtype=np.uint8).reshape(image.height,image.width,3)[:,:,::-1]
    if image.encoding == 'bgr8':
        return np.fromstring(image.data,dtype=np.uint8).reshape(image.height,image.width,3)    
    elif image.encoding == 'mono8':
        return np.fromstring(image.data,dtype=np.uint8).reshape(image.height,image.width)
    elif image.encoding == '32FC1':
        return np.fromstring(image.data,dtype=np.float32).reshape(image.height,image.width)
    else:
        raise Exception


class RvizWrapper:
    def __init__(self):
        self.pub = rospy.Publisher('visualization_marker', Marker)
        
    def draw_curve(self, arr, id, rgba = (0,1,0,1), width = .01, ns = "default_ns", frame_id = "base_footprint"):
        
        pose_array = gm.PoseArray()
        for (x,y,z) in arr:
            pose = gm.Pose()
            pose.position = gm.Point(x,y,z)
            pose_array.poses.append(pose)
            pose_array.header.frame_id = frame_id
            pose_array.header.stamp = rospy.Time.now()
                
        marker = Marker(type=Marker.LINE_STRIP,action=Marker.ADD)
        marker.header = pose_array.header
        marker.points = [pose.position for pose in pose_array.poses]
        marker.lifetime = rospy.Duration()
        marker.color = stdm.ColorRGBA(*rgba)
        marker.scale = gm.Vector3(width,width,width)
        marker.id = id
        marker.ns = ns
        self.pub.publish(marker)
        
        
        