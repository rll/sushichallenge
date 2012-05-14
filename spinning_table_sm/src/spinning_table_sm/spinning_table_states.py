PKG = "spinning_table_sm"
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import sensor_msgs.msg
import trajectory_msgs.msg as tm
import geometry_msgs.msg as gm
import smach_ros
from numpy import *
from numpy.linalg import norm
from collections import defaultdict
from misc_msgs.msg import *
from rotating_grasper.msg import *
from rotating_grasper.srv import *
from geometry_msgs.msg import Point,PointStamped,PolygonStamped
import numpy as np
import sys, os, yaml, subprocess
import rospkg

DIR = roslib.packages.get_pkg_dir(PKG, required=True) + "/config/"
stream = file(DIR+"config.yaml")
config = yaml.load(stream)

def make_fuerte_env():
    versionstr = sys.version[:3]
    return dict(
        ROS_MASTER_URI = os.environ["ROS_MASTER_URI"],
        PATH = "/opt/ros/fuerte/bin:%s"%os.environ["PATH"],
        ROS_VERSION = "fuerte",
        PYTHONPATH = "/opt/ros/fuerte/lib/python%s/dist-packages"%versionstr,
        ROS_PACKAGE_PATH = "/opt/ros/fuerte/share:/opt/ros/fuerte/stacks")


def make_tracker():    
    rp = rospkg.RosPack()
    pkg_path = rp.get_path("spinning_tabletop_detection")    
    p = subprocess.Popen(["%s/bin/test_tracker_ros"%pkg_path
                           ,"input_cloud:=/camera/rgb/points"
                           ,"--min_height=%s"%config["table_height_lower_bound"]
                           ,"--above_table_cutoff=%s"%config["above_table_cutoff"]                                                      
                           ], env = make_fuerte_env(), stdout = open('/dev/null','w'))
    return p

    

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


class TopicListener(object):
    "stores last message"
    last_msg = None
    def __init__(self,topic_name,msg_type):
        self.sub = rospy.Subscriber(topic_name,msg_type,self.callback)        
        
        rospy.loginfo('waiting for the first message: %s'%topic_name)
        while self.last_msg is None: rospy.sleep(.01)
        rospy.loginfo('ok: %s'%topic_name)
        
    def callback(self,msg):
        self.last_msg = msg


class TrajectoryControllerWrapper(object):

    def __init__(self, controller_name, listener):
        self.listener = listener
        
        self.joint_names = rospy.get_param("/%s/joints"%controller_name)

        self.n_joints = len(self.joint_names)
        
        msg = self.listener.last_msg
        self.ros_joint_inds = [msg.name.index(name) for name in self.joint_names]
        
        self.controller_pub = rospy.Publisher("%s/command"%controller_name, tm.JointTrajectory)        
        
        
    def get_joint_positions(self):
        msg = self.listener.last_msg
        return [msg.position[i] for i in self.ros_joint_inds]

    
    def goto_joint_positions(self, positions_goal):
        
        positions_cur = self.get_joint_positions()
        assert len(positions_goal) == len(positions_cur)

        duration = norm((r_[positions_goal] - r_[positions_cur])/self.vel_limits, ord=inf)
                
        jt = tm.JointTrajectory()
        jt.joint_names = self.joint_names
        jt.header.stamp = rospy.Time.now()
        
        jtp = tm.JointTrajectoryPoint()
        jtp.positions = positions_goal
        jtp.velocities = zeros(len(positions_goal))
        jtp.time_from_start = rospy.Duration(duration)
        
        jt.points = [jtp]
        self.controller_pub.publish(jt)

        rospy.loginfo("sleeping %.2f sec"%duration)
        rospy.sleep(duration)
            
    def follow_joint_traj(self, positions, duration = None):
        positions = np.r_[np.atleast_2d(self.get_joint_positions()), positions]
        positions[:,4] = np.unwrap(positions[:,4])
        positions[:,6] = np.unwrap(positions[:,6])
        positions, velocities, times = make_traj_with_limits(positions, self.vel_limits, self.acc_limits,smooth=True)                    
        self.follow_timed_joint_traj(positions, velocities, times)
def mirror_arm_joints(x):
    "mirror image of joints (r->l or l->r)"
    return r_[-x[0],x[1],-x[2],x[3],-x[4],x[5],-x[6]]


class Arm(TrajectoryControllerWrapper):

    L_POSTURES = dict(        
        untucked = [0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0],
        tucked = [0.06, 1.25, 1.79, -1.68, -1.73, -0.10, -0.09],
        up = [ 0.33, -0.35,  2.59, -0.15,  0.59, -1.41, -0.27],
        side = [  1.832,  -0.332,   1.011,  -1.437,   1.1  ,  -2.106,  3.074]
        )

    
    
    def __init__(self, lr,listener):
        TrajectoryControllerWrapper.__init__(self,"%s_arm_controller"%lr, listener)
        self.lr = lr
        self.lrlong = {"r":"right", "l":"left"}[lr]
        self.tool_frame = "%s_gripper_tool_frame"%lr
                
        self.cart_command = rospy.Publisher('%s_cart/command_pose'%lr, gm.PoseStamped)
        self.vel_limits = [0.42, 0.42,0.65,0.66,0.72, 0.62,0.72]

    def goto_posture(self, name):
        l_joints = self.L_POSTURES[name]        
        joints = l_joints if self.lr == 'l' else mirror_arm_joints(l_joints)
        self.goto_joint_positions(joints)

    def goto_joint_positions(self, positions_goal):
        positions_cur = self.get_joint_positions()        
        positions_goal = closer_joint_angles(positions_goal, positions_cur)
        TrajectoryControllerWrapper.goto_joint_positions(self, positions_goal)

        
    def set_cart_target(self, quat, xyz, ref_frame):
        ps = gm.PoseStamped()
        ps.header.frame_id = ref_frame
        ps.header.stamp = rospy.Time(0)
        ps.pose.position = gm.Point(*xyz);
        ps.pose.orientation = gm.Quaternion(*quat)
        self.cart_command.publish(ps)


def fix_angle(angle, center_point=math.pi):
    while angle > center_point + math.pi:
        angle = angle - 2*math.pi
    while angle < center_point - math.pi:
        angle = angle + 2*math.pi
    return angle

class Head(TrajectoryControllerWrapper):
    def __init__(self, listener):
        TrajectoryControllerWrapper.__init__(self,"head_traj_controller",listener)
        self.vel_limits = [1.,1.]
    def set_pan_tilt(self, pan, tilt):
        self.goto_joint_positions([pan, tilt])

        
        
        
        
        
        
####################################





class MoveArmToSide(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"])
        listener = TopicListener("joint_states",sensor_msgs.msg.JointState)
        self.rarm = Arm("r",listener)
        self.larm = Arm("l",listener)
        self.head = Head(listener)        
        

    def execute(self, userdata):
        rospy.sleep(1)
        self.larm.goto_posture('up')
        self.rarm.goto_posture('up')
        self.head.set_pan_tilt(0,.7)
        return "success"

class GatherDetections(smach.State):
    
    tracker = None
    
    def __init__(self,detector=None):
        smach.State.__init__(self, outcomes=["success", "failure"],output_keys=["cylinders"])
        self.detector = detector
        self.cylinders = defaultdict(list) #list of (center, radius)
        self.num_detections = 0
        

    def kill_tracker(self, *args):
        self.tracker.terminate()

    def execute(self, userdata):
        #TODO: start detector
        
        if self.tracker is None or self.tracker.poll() is not None:
            self.tracker = make_tracker()
        
        self.done = False
        print 'subscribing'
        rospy.Subscriber('/spinning_tabletop/cylinders',TrackedCylinders,self.handle_detection)
        sleep_time = 10
        print 'waiting for %d seconds' % sleep_time
        rospy.sleep(sleep_time)
        self.done = True

        maxLength = -1
        maxKey = -1
        for key in self.cylinders.keys():
            if len(self.cylinders[key]) > maxLength:
                maxLength = len(self.cylinders[key])
                maxKey = key
        if maxKey == -1:
            print 'no detections!'
            return "failure"
        print 'chose id %s with length %d' % (maxKey,maxLength)
        userdata.cylinders = self.cylinders[maxKey]
        return "success"

    def handle_detection(self,detection):
        if self.done: return
        for i in range(len(detection.ids)):
            pt = PointStamped()
            pt.header = detection.header
            pt.point.x = detection.xs[i]
            pt.point.y = detection.ys[i]
            pt.point.z = detection.zs[i] + detection.hs[i]
            self.cylinders[detection.ids[i]].append((pt,detection.rs[i], detection.hs[i]))
        self.num_detections += 1


class FitCircle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"],input_keys=["cylinders"],output_keys=["center","radius","rotation_rate","init_angle","init_time","object_radius", "object_height"])

    def execute(self, userdata):
        print 'fitting circle'
        times = []
        times_mat = mat(ones((len(userdata.cylinders),1)))
        x = mat(ones((len(userdata.cylinders),1)))
        y = mat(ones((len(userdata.cylinders),1)))
        z = ones(len(userdata.cylinders))
        r = ones(len(userdata.cylinders))
        h = zeros(len(userdata.cylinders))
        for i in range(len(userdata.cylinders)):
            x[i,0] = userdata.cylinders[i][0].point.x
            y[i,0] = userdata.cylinders[i][0].point.y
            z[i] = userdata.cylinders[i][0].point.z
            r[i] = userdata.cylinders[i][1]
            h[i] = userdata.cylinders[i][2]

            times.append(userdata.cylinders[i][0].header.stamp)
            times_mat[i,0] = userdata.cylinders[i][0].header.stamp.to_sec() - userdata.cylinders[0][0].header.stamp.to_sec()

        A = hstack([x, y, mat(ones(x.shape))])
        b = -(power(x,2)+power(y,2))
        a = asarray(linalg.lstsq(A,b)[0])
        xc = -.5 * a[0];
        yc = -.5 * a[1];
        zc = mean(z)
        center_radius = sqrt((a[0]**2+a[1]**2)/4-a[2])
        object_radius = mean(r)
        object_height = mean(h)
        R  =  center_radius + object_radius

        middle_ind = round(len(userdata.cylinders)/2.)
        print "len %d middle ind %d" % (len(userdata.cylinders),middle_ind)
        middle_angle = math.atan2(y[middle_ind,0]-yc,x[middle_ind,0]-xc)

        angles = mat(ones((len(userdata.cylinders),1)))

        print x.shape, y.shape, len(userdata.cylinders)
        for i in range(min([len(userdata.cylinders), len(x), len(y)])):
            angles[i,0] = fix_angle(math.atan2(y[i,0]-yc,x[i,0]-xc),middle_angle)
        # prev_angle = angles[0,0]
        # for i in range(len(userdata.cylinders)):
            # while angles[i,0] < prev_angle:
                # angles[i,0] = angles[i,0] + 2*math.pi
            # prev_angle = angles[i,0]
        A_angles = hstack([times_mat,mat(ones(angles.shape))])

        #print hstack([A_angles,angles])

        w_result = asarray(linalg.lstsq(A_angles,angles)[0])
        w = -w_result[0]
        print 'rotation rate: %.3f rad/s - one revolution in %.2f sec' % (w,2*math.pi/w)
        #w = 2 * math.pi / 30.

        userdata.center = Point(xc,yc,zc)
        userdata.radius = R
        userdata.object_radius = object_radius
        userdata.object_height = object_height
        userdata.rotation_rate = w
        userdata.init_angle = math.atan2(y[0,0]-yc,x[0,0]-xc)
        userdata.init_time = times[0]

        polygon_pub = rospy.Publisher('/fit_circle', geometry_msgs.msg.PolygonStamped)
        polygon1 = PolygonStamped()
        polygon1.header.stamp = rospy.Time.now()
        polygon1.header.frame_id = 'base_footprint'
        polygon2 = PolygonStamped()
        polygon2.header.stamp = rospy.Time.now()
        polygon2.header.frame_id = 'base_footprint'
        for angle in linspace(0,2*math.pi,math.pi/8.):
            pt1 = Point(xc+center_radius+math.cos(angle),yc+center_radius+math.sin(angle),zc)
            pt2 = Point(xc+R+math.cos(angle),yc+R+math.sin(angle),zc)
            polygon1.polygon.points.append(pt1)
            polygon2.polygon.points.append(pt2)
        polygon_pub.publish(polygon1)
        polygon_pub.publish(polygon2)

        print 'got center (%.3f,%.3f,%.3f), radius %.3f + %.3f = %.3f' % (xc,yc,zc,center_radius,object_radius,R)
        return "success"

class ExecuteGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure","missed"],input_keys=["center","radius","rotation_rate","init_angle","init_time","object_radius","object_height"])

    def execute(self, userdata):
        center = userdata.center
        radius = userdata.radius
        rotation_rate = userdata.rotation_rate
        init_angle = userdata.init_angle
        init_time = userdata.init_time

        command = RotatingGrasp()
        command.header.stamp = userdata.init_time
        command.header.frame_id = 'base_footprint'
        command.center = Point()
        command.center.x = center.x
        command.center.y = center.y
        command.center.z = center.z

        command.initial = Point()
        command.initial.x = center.x + math.cos(init_angle)*radius
        command.initial.y = center.y - math.sin(init_angle)*radius
        command.initial.z = center.z

        command.object_height = userdata.object_height+config["above_table_cutoff"]
        command.object_radius = userdata.object_radius

        command.rotation_rate = rotation_rate

        print "radius,height",userdata.object_radius, userdata.object_height
        if userdata.object_radius < .06:
            print "CUP!"
            command.outward_angle = 0
        elif userdata.object_radius < .12:
            print "BOWL"
            command.outward_angle = math.pi/4
        else:
            print "PLATE"
            command.outward_angle = math.pi/2 - math.pi/8
        print 'waiting for service'
        rospy.wait_for_service('rotating_grasper')
        server = rospy.ServiceProxy('rotating_grasper', RotatingGrasper)
        try:
            print 'calling...'
            print command
            success = True
            response = server(command)
            server_success = response.success

            if not server_success:
                print "Service call failed"
                return "failure"


            msg = rospy.wait_for_message('/joint_states',sensor_msgs.msg.JointState)
            gripper_joint_angle = msg.position[msg.name.index("r_gripper_joint")]
            grab_success = (gripper_joint_angle > .002)

            if not grab_success:
                print "missed!"
                return "missed"

            return "success"

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return "failure"