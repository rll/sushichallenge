PKG = "spinning_table_sm"
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros

from numpy import *

from collections import defaultdict

from misc_msgs.msg import *

from rotating_grasper.msg import *
from rotating_grasper.srv import *

from geometry_msgs.msg import Point,PointStamped,PolygonStamped

def fix_angle(angle, center_point=math.pi):
    while angle > center_point + math.pi:
        angle = angle - 2*math.pi
    while angle < center_point - math.pi:
        angle = angle + 2*math.pi
    return angle

class MoveArmToSide(smach.State):
    def __init__(self, tasks):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.tasks = tasks

    def execute(self, userdata):
        try:
            self.tasks.move_arm_to_side('right_arm')
        except:
            return "failure"
        return "success"

class GatherDetections(smach.State):
    def __init__(self,detector=None):
        smach.State.__init__(self, outcomes=["success", "failure"],output_keys=["cylinders"])
        self.detector = detector
        self.cylinders = defaultdict(list) #list of (center, radius)
        self.num_detections = 0
    
    def execute(self, userdata):
        #TODO: start detector
        print 'subscribing'
        rospy.Subscriber('/spinning_tabletop/cylinders',TrackedCylinders,self.handle_detection)
        sleep_time = 15
        print 'waiting for %d seconds' % sleep_time
        rospy.sleep(sleep_time)
            
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
        for i in range(len(detection.ids)):
            pt = PointStamped()
            pt.header = detection.header
            pt.point.x = detection.xs[i]
            pt.point.y = detection.ys[i]
            pt.point.z = detection.zs[i] + detection.hs[i]
            self.cylinders[detection.ids[i]].append((pt,detection.rs[i]))
        self.num_detections += 1
        
        
class FitCircle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"],input_keys=["cylinders"],output_keys=["center","radius","rotation_rate","init_angle","init_time"])
    
    def execute(self, userdata):
        print 'fitting circle'
        times = []
        times_mat = mat(ones((len(userdata.cylinders),1)))
        x = mat(ones((len(userdata.cylinders),1)))
        y = mat(ones((len(userdata.cylinders),1)))
        z = ones(len(userdata.cylinders))
        r = ones(len(userdata.cylinders))
        for i in range(len(userdata.cylinders)):
            x[i,0] = userdata.cylinders[i][0].point.x
            y[i,0] = userdata.cylinders[i][0].point.y
            z[i] = userdata.cylinders[i][0].point.z
            r[i] = userdata.cylinders[i][1]
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
        R  =  center_radius + object_radius
        
        middle_ind = round(len(userdata.cylinders)/2.)
        print "len %d middle ind %d" % (len(userdata.cylinders),middle_ind)
        middle_angle = math.atan2(y[middle_ind,0]-yc,x[middle_ind,0]-xc)
        
        angles = mat(ones((len(userdata.cylinders),1)))
        for i in range(len(userdata.cylinders)):
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
        smach.State.__init__(self, outcomes=["success", "failure"],input_keys=["center","radius","rotation_rate","init_angle","init_time"])
    
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
        
        command.rotation_rate = rotation_rate
        command.outward_angle = math.pi/2.3
        
        print 'waiting for service'
        rospy.wait_for_service('rotating_grasper')
        server = rospy.ServiceProxy('rotating_grasper', RotatingGrasper)
        try:
            print 'calling...'
            print command
            success = True
            response = server(command)
            success = response.success
            if success:
                print 'success!'
                return "success"
            else:
                print 'failed :('
                return "failure"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return "failure"