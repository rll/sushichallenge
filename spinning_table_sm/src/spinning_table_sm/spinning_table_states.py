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

from geometry_msgs.msg import Point,PointStamped

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
        
        A = hstack([x, y, mat(ones(x.shape))])
        b = -(power(x,2)+power(y,2))
        a = asarray(linalg.lstsq(A,b)[0])
        xc = -.5 * a[0];
        yc = -.5 * a[1];
        zc = mean(z)
        R  =  sqrt((a[0]**2+a[1]**2)/4-a[2]) + mean(r);
        
        userdata.center = Point(xc,yc,zc)
        userdata.radius = R
        userdata.rotation_rate = 2 * math.pi / 30 #TODO: fix
        userdata.init_angle = math.atan2(y[0,0]-yc,x[0,0]-xc)
        userdata.init_time = times[0]
        
        print 'got center (%.3f,%.3f,%.3f) and radius %.3f' % (xc,yc,zc,R)
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