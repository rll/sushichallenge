"""
Author: Lorenzo Riano <lorenzo.riano@gmail.com>
"""
PKG = "sushi_sm"
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
#import pr2_python
import yaml

from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_python import transform_listener, pointclouds
import dynamic_reconfigure.client
from sushi_sm.world_definitions import WorldState, PickableObject
from sushi_sm.grab_plate import grasp_plate
from pr2_python.find_free_space import free_spots
from geometry_msgs.msg import Point
from pr2_python.transform_listener import transform_point
from spinning_table_sm.sm import create_spinning_table_sm



import sys, subprocess

DIR = roslib.packages.get_pkg_dir(PKG, required=True) + "/config/"
stream = file(DIR+"poses_icra.yaml")
poses = yaml.load(stream)

stream = file(DIR+"furniture_dims.yaml")
furniture_dims = yaml.load(stream)

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
                           ,"--min_height=%s"%config["min_filter_height"]
                           ,"--max_height=%s"%config["max_filter_height"]
                           ,"--above_table_cutoff=%s"%config["above_table_cutoff"]                                                      
                           ], env = make_fuerte_env(), stdout = open('/dev/null','w'))
    return p


class NavigateTo(smach.State):
    """Navigate to a desired position.

    Constructor: NavigateTo(world_state, base_mover, x,y,theta)
    world_state: a WorldState instance
    base_mover is a pr2_python.base.Base instance
    x,y,theta are in the /map frame of reference (see pr2_python)
    """
    def __init__(self, world, base_mover, x, y, theta):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.x = x
        self.y = y
        self.theta = theta
        self.base = base_mover
        self.world_state = world

    def execute(self, userdata):
        pos = (self.x, self.y, self.theta)
        rospy.loginfo("%s: going to %s", self.__class__.__name__,
                     pos)
        try:
            self.base.move_to(pos[0], pos[1], pos[2])
            return "success"
        except:
            return "failure"

class Resetter(smach.State):
    def __init__(self, interface):
        smach.State.__init__(self, outcomes = ["success"])
        self.interface = interface
    def execute(self, userdata):
        self.interface.reset()
        psi = get_planning_scene_interface()
        psi.reset()
        return "success"

class Detect(smach.State):
    """Detect objects and add them to the shared world
    
    Constructor: Detect(world_state, location, detector, head_point)
    world_state: a WorldState instance
    location: 
    detector: pr2_tasks TablewareDetection
    head_point: a PointStamped to move the head to. None if no head movement.
    """

    def __init__(self, world, location, detector, head_point = None):
        smach.State.__init__(self, outcomes=["success", "failure", "no object"])
        self.detector = detector
        self.head_point = head_point
        self.world = world
        self.location = location

    def execute(self, userdata):
        det = self.detector.detect_objects(point_head_at=self.head_point)
        if not det.pickup_goals:
            rospy.loginfo('Nothing to pick up!')
            return "no object"
        for goal in det.pickup_goals:
            obj = PickableObject(goal, self.location)
            self.world.add_item(obj)

        rospy.loginfo("World has now: %s", self.world.items)
        return "success"

class PickUp(smach.State):
    def __init__(self, world, tasks):
        smach.State.__init__(self, outcomes=["success", "failure", "no_free_arm"])
        self.world = world
        self.tasks = tasks

    def execute(self, userdata):
        if len(self.world.free_arms) == 0:
            return "no_free_arm"
        goal = self.world.pickup_next
        
        pose = (goal.mappose.pose.position.x,
                goal.mappose.pose.position.y,
                goal.mappose.pose.position.z
               )
        graspable = goal.graspable        
        
        graspable.arm_name = self.world.free_arms[0]
        goal.arm_name = graspable.arm_name
        rospy.loginfo("Using arm %s", goal.arm_name)
        try:
            self.tasks.go_and_pickup(goal.arm_name,pose,goal.label)
        except Exception, e:
            rospy.logerr("Error while executing pickup: %s", e)
            self.world.failed_to_pickup = goal
            return "failure"
       
        self.world.items[goal.location].remove(goal)
        self.world.object_in_hands[goal.arm_name] = goal
        self.world.free_arms.remove(goal.arm_name)
        self.tasks.move_arms_to_side()
        self.world.failed_to_pickup = None
        return "success"

class ChooseIfPutDown(smach.State):
    """
    If the robot has an object in the gripper, returns go_on, otherwise return
    stop. This class is useful to decide what to do after failures.
    """
    def __init__(self, world):
        smach.State.__init__(self, outcomes=["go_on", "stop"])
        self.world = world
        
    def execute(self, userdata):
        if len(self.world.object_in_hands) == 0:
            return "stop"
        else:
            return "go_on"

class ChooseIfGoLocation(smach.State):
    """
    If there are objects to pickup, returns go_on, otherwise returns
    stop.
    """
    def __init__(self, world, location):
        smach.State.__init__(self, outcomes=["go_on", "stop"])
        self.world = world
        self.location = location
        
    def execute(self, userdata):
        if len(self.world.items[self.location]) == 0:
            rospy.loginfo("No more objects at %s", self.location)
            return "stop"
        else:
            rospy.loginfo("There's still objects at %s", self.location)
            return "go_on"

class RemoveItem(smach.State):
    """
    Remove an item which is marked failed_to_pickup
    """
    def __init__(self, world):
        smach.State.__init__(self, outcomes=["success"])
        self.world = world
    
    def execute(self, userdata):
        goal = self.world.failed_to_pickup
        if goal is None:
            rospy.logwarn("Attempting to remove a None item")
            return "success"
        self.world.remove_item(goal)
        self.world.failed_to_pickup
        return "success"

class PickUpSimple(smach.State):
    def __init__(self, world=None, pickplace=None, tasks=None, find_box=None, base_mover=None):
        smach.State.__init__(self, outcomes=["success", "failure", "no_free_arm"])
        self.world = world
        self.pickplace = pickplace
        self.tasks = tasks
        self.find_box = find_box
        self.base = base_mover
        
        self.cylinders = defaultdict(list) #list of (center, radius)
        self.num_detections = 0

    def kill_tracker(self, *args):
        self.tracker.terminate()

    def grasp_plate(self, goal):
        x = goal.pose_stamped.pose.position.x
        y = goal.pose_stamped.pose.position.y
        z = goal.pose_stamped.pose.position.z
        try:
            self.base.move_manipulable_pose(x, y, z, 
                    try_hard = True, group="torso")
        except Exception as e:
            rospy.logerr("Got an error: %s", e)
            return "failure"
        if len(self.world.free_arms) == 2:
            lr_force = None
        else:
            lr_force = self.world.free_arms[0]
        listener = transform_listener.get_transform_listener()
        try:
            if self.tracker is None or self.tracker.poll() is not None:
                self.tracker = make_tracker()
        
            self.done = False
            print 'subscribing'
            rospy.Subscriber('/spinning_tabletop/cylinders',TrackedCylinders,self.handle_detection)
            sleep_time = 11
            print 'waiting for %d seconds' % sleep_time
            rospy.sleep(sleep_time)
            self.done = True
            self.tracker.terminate()
            
            #key = self.cylinders.keys()[0]
            min_dist = 1000000
            min_key = None
            for key in self.cylinders.keys():
                pt = self.cylinders[key][0].point
                dist = math.sqrt((pt.x-x)**2 + (pt.y-y)**2)
                if dist < min_dist:
                    min_dist = dist
                    min_key = key
            
            cylinder = self.cylinders[min_key][0]
            
            return grasp_plate_from_cylinder(cylinder,listener,lr_force=lr_force)

            # pointcloud = goal.graspable.target.cluster
            # box = self.find_box(pointcloud)
            # return grasp_plate(box, listener, lr_force=lr_force)
        except Exception as e:
            rospy.logerr("Got an error: %s", e)
            return False        

    def handle_detection(self,detection):
        if self.done: return
        for i in range(len(detection.ids)):
            pt = PointStamped()
            pt.header = detection.header
            pt.point.x = detection.xs[i]
            pt.point.y = detection.ys[i]
            pt.point.z = detection.zs[i] - detection.hs[i]
            self.cylinders[detection.ids[i]].append((pt,detection.rs[i], detection.hs[i]))
        self.num_detections += 1
        self.done = True
        
    def execute(self, userdata):
        if len(self.world.free_arms) == 0:
            return "no_free_arm"
        goal = self.world.pickup_next
        if goal.label == "plate":
            ret = self.grasp_plate(goal)
            if ret:
                return "success"
            else:
                return "failure"
        graspable = goal.graspable        

        grabbed_it = False
        for free_arm in self.world.free_arms:        
            graspable.arm_name = free_arm
            goal.arm_name = graspable.arm_name
            rospy.loginfo("Using arm %s", goal.arm_name)
            try:
                self.pickplace.pickup(graspable)
                grabbed_it = True
                break
            except Exception as e:
                rospy.logerr("Problem: %s ", e)
                pass
        
        if not grabbed_it:
            self.world.failed_to_pickup = goal
            return "failure"
       
        self.world.items[goal.location].remove(goal)
        self.world.object_in_hands[goal.arm_name] = goal
        self.world.free_arms.remove(goal.arm_name)
        self.tasks.move_arms_to_side()
        self.world.failed_to_pickup = None
        return "success"

class ChooseItem(smach.State):
    def __init__(self, world, location):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.world = world
        self.location = location

    def execute(self, userdata):
        try:
            goal = self.world.items[self.location][0]
        except IndexError:
            return "failure"
        self.world.pickup_next = goal
        return "success"

class MoveToPickable(smach.State):
    def __init__(self, world, base_mover):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.world = world
        self.base = base_mover
    
    def execute(self, userdata):
        #rospy.logwarn("PEZZ HACKING!!")
        #return "success"

        goal = self.world.pickup_next
        
        pose = goal.object_pose_stamped
        mappose = transform_listener.transform_pose_stamped("/map",pose)

        x = mappose.pose.position.x
        y = mappose.pose.position.y
        z = mappose.pose.position.z
        rospy.loginfo("Trying to go to %s", (x,y,z))
        try:
            self.base.move_manipulable_pose(x, y, z, 
                    try_hard = True, group="torso")
        except:
            return "failure"
        return "success"

class MoveToReachable(smach.State):
    def __init__(self, world, base_mover, pos):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.world = world
        self.base = base_mover
        self.pos = pos
    
    def execute(self, userdata):
        x = self.pos[0]
        y = self.pos[1]
        z = self.pos[2]
        rospy.loginfo("Trying to go to %s", (x,y,z))
        try:
            self.base.move_manipulable_pose(x, y, z, 
                    try_hard = True, group="torso")
        except:
            raise
            return "failure"
        return "success"

class RoundRobinTableMover(smach.State):
    def __init__(self, world, base_mover):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.world = world
        self.base = base_mover
    
    def execute(self, userdata):
        location = self.world.get_next_round_robin_location()
        rospy.loginfo("next location: %s", location)
        pos = poses[location]
        
        x = pos[0]
        y = pos[1]
        z = pos[2]
        rospy.loginfo("Trying to go to %s", (x,y,z))
        try:
            self.base.move_to(x, y, z,)
        except:
            raise
            return "failure"
        return "success"

class MoveArmsToSide(smach.State):
    def __init__(self, world, tasks):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.world = world
        self.tasks = tasks

    def execute(self, userdata):
        try:
            self.tasks.move_arms_to_side()
        except:
            return "failure"
        return "success"

class PlaceDown(smach.State):
    def __init__(self, world, placer_l, placer_r, pos):
        smach.State.__init__(self, outcomes=["success", "failure",
            "still_holding"])
        self.world = world
        self.placer_l = placer_l
        self.placer_r = placer_r
        self.pos = pos

    def execute(self, userdata):
        
        if len(self.world.object_in_hands) == 0:
            rospy.logerr("Hands are empty! are they?")
            return "success"
        
    
        arm = self.world.object_in_hands.keys()[0]
        if arm == "right_arm":
            placer = self.placer_r
        else:
            placer = self.placer_l        
        try:
            placer.place(self.pos[0], self.pos[1], self.pos[2])
        except:
            return "failure"
    
        del self.world.object_in_hands[arm]
        self.world.free_arms.append(arm)
        
        rospy.loginfo("World: free_arms: %s", self.world.free_arms)
        rospy.loginfo("World: object_in_hands: %s", self.world.object_in_hands)
        if len(self.world.object_in_hands) == 0:
            return "success"
        else:
            return "still_holding"

class FakeObject(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=["success"])
        self.world = world
    
    def execute(self, userdata):
        self.world.object_in_hands['right_arm'] = "bogus"
        return "success"

class PlaceDownFreeSpace(smach.State):
    def __init__(self, world, placer_l, placer_r, detector, find_box):
        smach.State.__init__(self, outcomes=["success", "failure",
            "still_holding"])
        
        self.find_box = find_box
        self.world = world
        self.placer_l = placer_l
        self.placer_r = placer_r
        self.detector = detector
    
    def execute(self, userdata):               
        if len(self.world.object_in_hands) == 0:
            rospy.logerr("Hands are empty! are they?")
            return "success"
        arm = self.world.object_in_hands.keys()[0]
        if arm == "right_arm":
            placer = self.placer_r
        else:
            placer = self.placer_l
        
        det = self.detector.detect_objects(point_head_at=None)

        table = det.table
        table_dims = (table.x_max - table.x_min, table.y_max - table.y_min)
        rospy.loginfo("Table dims: %s", table_dims)
            
        objs = []
        for goal in det.pickup_goals:
            pointcloud = goal.target.cluster
            box = self.find_box(pointcloud)
            rospy.loginfo("----- Found label %s", goal.label)
            dims = box.box_dims
            pos = (box.pose.pose.position.x, box.pose.pose.position.y)
            obj = ( (pos[0] - dims.x/2 - table.x_min, pos[1] - dims.y/2 - table.y_min), (dims.x, dims.y))
            rospy.loginfo("Obj tuple: %s", obj)
            objs.append(obj)
        
        obj_dim = (0.15, 0.15)
        spot = free_spots(table_dims, [obj_dim], 0.01, objs)[0]
        spot = (spot[0] + table.x_min - obj_dim[0]/2., 
                spot[1] + table.y_min - obj_dim[1]/2.)        
        rospy.loginfo("Spot: %s", spot)
        
        pos = Point()
        pos.x = spot[0]
        pos.y = spot[1]
        pos.z = 0.0
        
        newpos = transform_point("/map", table.pose.header.frame_id, pos)
        newpos.z = 1.0
        rospy.loginfo("Putting down at: %s", newpos)
        try:
            placer.place(newpos.x, newpos.y, newpos.z)
        except Exception, e:
            rospy.logerr("Error while putting down, error is: %s", e)
            return "failure"
    
        del self.world.object_in_hands[arm]
        self.world.free_arms.append(arm)
        
        rospy.loginfo("World: free_arms: %s", self.world.free_arms)
        rospy.loginfo("World: object_in_hands: %s", self.world.object_in_hands)
        if len(self.world.object_in_hands) == 0:
            return "success"
        else:
            return "still_holding"
        

class LookAtTable(smach.State):        
    def __init__(self, world, head):
        smach.State.__init__(self, outcomes=["success"])
        self.world = world
        self.head = head
        self.filter_x = dynamic_reconfigure.client.Client("/pcl_filters/psx")
        self.filter_y = dynamic_reconfigure.client.Client("/pcl_filters/psy")
        self.filter_z = dynamic_reconfigure.client.Client("/pcl_filters/psz")
    
    def execute(self, userdata):
        height = furniture_dims["table"]["height"]
        width = furniture_dims["table"]["width"]
        
        x = 1.0
        y = 0.0
        z = height
        self.head.look_at_relative_point(x,y,z)
        
        depth = 1.2
        params = {'filter_limit_min' : 0, 'filter_limit_max' : depth,
                  'input_frame':"/base_footprint"}
        self.filter_x.update_configuration(params)
        
        params = {'filter_limit_min' : -width/2., 'filter_limit_max' : width/2.,
                  'input_frame':"/base_footprint"}
        self.filter_y.update_configuration(params)
        
        params = {'filter_limit_min' : height-0.2, 'filter_limit_max' : height+0.5,
                  'input_frame':"/base_footprint"}    
        self.filter_z.update_configuration(params)
        return "success"

class LookAtObjectPickup(smach.State):        
    def __init__(self, world, head):
        smach.State.__init__(self, outcomes=["success"])
        self.world = world
        self.head = head
                
        self.filter_x = dynamic_reconfigure.client.Client("/pcl_filters/psx")
        self.filter_y = dynamic_reconfigure.client.Client("/pcl_filters/psy")
        self.filter_z = dynamic_reconfigure.client.Client("/pcl_filters/psz")
    
    def execute(self, userdata):
        target = self.world.pickup_next
        pos = transform_listener.transform_pose_stamped("/map", 
                                                        target.pose_stamped)
                
        x = pos.pose.position.x
        y = pos.pose.position.y
        z = pos.pose.position.z

        self.head.look_at_map_point(x,y,z)
        
        depth = x + .20
        params = {'filter_limit_min' : 0, 'filter_limit_max' : depth,
                  'input_frame':"/map"}
        self.filter_x.update_configuration(params)
        
        width = 0.50
        params = {'filter_limit_min' : y-width/2., 'filter_limit_max' : y+width/2.,
                  'input_frame':"/map"}
        self.filter_y.update_configuration(params)
        
        height = 0.50
        params = {'filter_limit_min' : z-height/2, 'filter_limit_max' : z+height/2,
                  'input_frame':"/map"}    
        self.filter_z.update_configuration(params)
        return "success"
      
class LookAtShelf(smach.State):        
    def __init__(self, world, head, shelf_number):
        smach.State.__init__(self, outcomes=["success"])
        self.world = world
        self.head = head
        self.shelf_number = shelf_number
        self.filter_x = dynamic_reconfigure.client.Client("/pcl_filters/psx")
        self.filter_y = dynamic_reconfigure.client.Client("/pcl_filters/psy")
        self.filter_z = dynamic_reconfigure.client.Client("/pcl_filters/psz")
    
    def execute(self, userdata):
        shelves = furniture_dims["shelves"]
        which_shelf = self.shelf_number
        max_height = shelves["heights"][which_shelf]['max']
        min_height = shelves["heights"][which_shelf]['min']
        width = shelves['width']
        depth = shelves['depth']
        
        x = 0.5
        y = 0.0
        z = min_height
        self.head.look_at_relative_point(x,y,z)
        
        params = {'filter_limit_min' : 0, 'filter_limit_max' : 1.0 + depth,
                 'input_frame':"/base_footprint"}
        self.filter_x.update_configuration(params)
        
        params = {'filter_limit_min' : -width/2., 'filter_limit_max' : width/2.,
                  'input_frame':"/base_footprint"}                  
        self.filter_y.update_configuration(params)
        
        params = {'filter_limit_min' : min_height-0.2, 'filter_limit_max' : max_height -0.05,
                  'input_frame':"/base_footprint"}                      
        self.filter_z.update_configuration(params)
        return "success"    

class EnableFullView(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=["success"])
        self.world = world
        self.filter_x = dynamic_reconfigure.client.Client("/pcl_filters/psx")
        self.filter_y = dynamic_reconfigure.client.Client("/pcl_filters/psy")
        self.filter_z = dynamic_reconfigure.client.Client("/pcl_filters/psz")
    
    def execute(self, userdata):
        params = {'filter_limit_min' : -5, 'filter_limit_max' : 5.0,
                  'input_frame':"/base_footprint"}
        self.filter_x.update_configuration(params)
        
        params = {'filter_limit_min' : -5., 'filter_limit_max' : 5.,
                  'input_frame':"/base_footprint"}
        self.filter_y.update_configuration(params)
        
        params = {'filter_limit_min' : -5, 'filter_limit_max' : 5,
                  'input_frame':"/base_footprint"}    
        self.filter_z.update_configuration(params)
        return "success"    

def create_pickup_sm(inspector, location):
    sm = smach.StateMachine(outcomes=["success",
                                      "failure",
                                      ])
    with sm:
        smach.StateMachine.add("choose",
                inspector(ChooseItem, location=location),
                transitions = {"success":"pickup_simple",
                    "failure":"success"}
                )
        
        smach.StateMachine.add("pickup_simple",
                inspector(PickUpSimple),
                transitions={"success": "choose",
                             "no_free_arm": "success",
                             "failure": "pickup"})
        
        smach.StateMachine.add("pickup",
                    inspector(PickUp),
                    transitions = {"success":"choose",
                                   "no_free_arm":"success",
                                   "failure":"failure"
                                  }
                    )
    return sm

def create_pickup_complex_sm(inspector, location):
    sm = smach.StateMachine(outcomes=["success",
                                      "failure",
                                      ])
    with sm:
        smach.StateMachine.add("choose",
                inspector(ChooseItem, location=location),
                transitions = {"success":"lookat",
                    "failure":"success"}
                )
        
        smach.StateMachine.add("lookat",
                inspector(LookAtObjectPickup),
                transitions={"success":"pickup"})
        
        smach.StateMachine.add("pickup",
                    inspector(PickUp),
                    transitions = {"success":"choose",
                                   "no_free_arm":"full_view",
                                   "failure":"full_view_failure"
                                  }
                    )
        smach.StateMachine.add("full_view",
                    inspector(EnableFullView),
                    transitions = {"success":"success"}
        )
        
        smach.StateMachine.add("full_view_failure",
                    inspector(EnableFullView),
                    transitions = {"success":"failure"}
        )
    return sm

def create_detect_sm(inspector, location, pos, head_mover):
    """
    pos: (x,y,th) where to move the robot
    location: semantic label
    head_mover: a lookat state
    """
    sm = smach.StateMachine(outcomes=["success",
                                      "failure",
                                      "no_object"])
    with sm:
        smach.StateMachine.add("move_" + location,
                inspector(NavigateTo,
                    x=pos[0],y=pos[1],theta=pos[2]
                    ),
                transitions = {"success":"move_head",
                               "failure":"failure"
                              }
                )
        smach.StateMachine.add("move_head",
            head_mover,
            transitions = {"success":"detect"})
         
        smach.StateMachine.add("detect",
                inspector(Detect, location=location),
                transitions = {"success":"success",
                               "failure":"failure",
                               "no object":"no_object"}
                )
    return sm

def create_clean_table_sm(inspector):
    sm = smach.StateMachine(outcomes=["success",
                                      "failure"])
    location = "table"
    pos = poses["dirty_table_putdown"]
    
    with sm:        
        
        detect_sm = create_detect_sm(inspector, location, poses["table_top_edge"],
                                     inspector(LookAtTable))

        smach.StateMachine.add("detect_table",
                detect_sm,
                transitions = {"success":"choose",
                               "failure":"failure",
                               "no_object":"success"})
        
        smach.StateMachine.add("choose",
                inspector.instanciate(
                    ChooseItem, location=location),
                transitions = {"success":"pickup",
                    "failure":"success"}
                )
        
        pickup_sm = create_pickup_sm(inspector, location)        
        smach.StateMachine.add("pickup",
                pickup_sm,
                transitions = {"success":"move_dirty_table",
                              "failure":"remove_item",
                              }
            )
        
        smach.StateMachine.add("remove_item",
                               inspector(RemoveItem),
                               transitions={"success":"choose_if_putdown"}
                               )
        
        #this is semi-failure condition
        smach.StateMachine.add("choose_if_putdown",
                inspector.instanciate(
                    ChooseIfPutDown),
                transitions = {"stop":"choose_go_back",
                               "go_on":"move_dirty_table"}
        )
        
                #this is semi-failure condition
        smach.StateMachine.add("choose_go_back",
                inspector.instanciate(
                    ChooseIfGoLocation, location=location),
                transitions = {"stop":"failure",
                               "go_on":"choose"}
        )
        
        smach.StateMachine.add("move_dirty_table",
                inspector.instanciate(
                    MoveToReachable, pos = pos),
                transitions = {"success":"putdown",
                               "failure":"failure"
                              }
        )

        smach.StateMachine.add("move_arms_side",
                inspector.instanciate(
                    MoveArmsToSide),
                transitions = {"success":"putdown",
                               "failure":"putdown"
                              }
        )
        
        smach.StateMachine.add("putdown",
                inspector.instanciate(
                    PlaceDown, pos = pos),
                transitions = {"success":"reset",
                               "failure":"putdown",
                               "still_holding":"move_arms_side"
                              }
                )
#        smach.StateMachine.add("putdown",
#                inspector.instanciate(
#                    PlaceDownFreeSpace),
#                transitions = {"success":"reset",
#                               "failure":"putdown",
#                               "still_holding":"move_arms_side"
#                              }
#                )
        
        smach.StateMachine.add("reset",
                inspector.instanciate(
                    Resetter),
                transitions = {"success":"move_arms_side2"}
                )
        
        smach.StateMachine.add("move_arms_side2",
                inspector.instanciate(
                    MoveArmsToSide),
                transitions = {"success":"detect_table",
                       "failure":"detect_table"
                      }
        )
                               
    return sm  

def create_detect_shelves_sm(inspector):
    location = "shelf"    
    sm = smach.StateMachine(outcomes=["success",
                                      "failure",
                                      "no_object"])
    shelves = furniture_dims['shelves']
        
    with sm:        
        for i in xrange(len(shelves["heights"])):
            detect_sm = create_detect_sm(inspector, location, poses["shelf"],
                                         head_mover = inspector(LookAtShelf,
                                                                shelf_number=i)
                                         )
            if i == len(shelves["heights"]) - 1:
                next_s = "success"
                next_o = "success"
            else:
                next_s = "detect_shelf_"+str(i+1)
                next_o = "detect_shelf_"+str(i+1)
            smach.StateMachine.add("detect_shelf_"+str(i),
                                   detect_sm,
                                   transitions={"success":next_s,
                                                "failure":"failure",
                                                "no_object":next_o})
    return sm
        

def create_setup_table_sm(inspector):
    sm = smach.StateMachine(outcomes=["success",
                                      "failure"])
    location = "shelf"    
    with sm:
        pos = poses["shelf"]
        smach.StateMachine.add("move_shelf",
                inspector(NavigateTo,
                    x=pos[0],y=pos[1],theta=pos[2]
                    ),
                transitions = {"success":"detect_shelf",
                               "failure":"failure"
                              }
        )
        
        detect_sm = create_detect_shelves_sm(inspector)        
        smach.StateMachine.add("detect_shelf",
                detect_sm,
                transitions = {"success":"choose",
                               "failure":"failure",
                               "no_object":"success"}
        )
        
        smach.StateMachine.add("move_shelf2",
                inspector(NavigateTo,
                    x=pos[0],y=pos[1],theta=pos[2]
                    ),
                transitions = {"success":"choose",
                               "failure":"failure"
                              }
        )
        
        smach.StateMachine.add("choose",
                inspector.instanciate(
                    ChooseItem, location=location),
                transitions = {"success":"pickup",
                    "failure":"success"}
                )
        
#        smach.StateMachine.add("full_view",
#                               inspector(EnableFullView),
#                               transitions={"success":"pickup"}
#        )
        
        pickup_sm = create_pickup_complex_sm(inspector, location)        
        smach.StateMachine.add("pickup",
                pickup_sm,
                transitions = {"success":"move_round_robin",
                              "failure":"remove_item",
                              }
        )
        
        smach.StateMachine.add("remove_item",
                               inspector(RemoveItem),
                               transitions={"success":"choose_if_putdown"}
                               )
        
        #this is semi-failure condition
        smach.StateMachine.add("choose_if_putdown",
                inspector.instanciate(
                    ChooseIfPutDown),
                transitions = {"stop":"choose_go_back",
                               "go_on":"move_round_robin"}
        )
        
                #this is semi-failure condition
        smach.StateMachine.add("choose_go_back",
                inspector.instanciate(
                    ChooseIfGoLocation, location=location),
                transitions = {"stop":"failure",
                               "go_on":"move_shelf2"}
        )
        
        smach.StateMachine.add("move_round_robin",
                inspector(RoundRobinTableMover),
                transitions = {"success":"putdown",
                               "failure":"failure"
                              }
        )

        smach.StateMachine.add("move_arms_side",
                inspector.instanciate(
                    MoveArmsToSide),
                transitions = {"success":"putdown",
                               "failure":"putdown"
                              }
        )

        smach.StateMachine.add("putdown",
                inspector.instanciate(
                    PlaceDownFreeSpace),
                transitions = {"success":"reset",
                               "failure":"putdown",
                               "still_holding":"move_arms_side"
                              }
        )
                
        smach.StateMachine.add("reset",
                inspector.instanciate(
                    Resetter),
                transitions = {"success":"move_arms_side2"}
        )
        
        smach.StateMachine.add("move_arms_side2",
                inspector.instanciate(
                    MoveArmsToSide),
                transitions = {"success":"detect_shelf",
                       "failure":"detect_shelf"
                      }
        )
                               
    return sm  
       
def create_spinning_table_clear_sm(inspector):
    sm = smach.StateMachine(outcomes=["success",
                                      "failure"])
    location = "rotating_table"
    pos = poses[location]    
    with sm:
        smach.StateMachine.add("move_to_rotating_table",
            inspector(NavigateTo, x=pos[0], y = pos[1], theta=pos[2]),
            transitions = {"success":"grab_stuff",
                           "failure":"failure"}
            )
        
        grab_stuff_sm = create_spinning_table_sm()
        smach.StateMachine.add("grab_stuff",
                               grab_stuff_sm,
                               transitions={"success":"move_arms_side",
                                            "failure":"failure"}
            )
        
        smach.StateMachine.add("move_arms_side",
                               inspector(MoveArmsToSide),
                               transitions={"success":"fake_object",
                                            "failure":"failure"}
                               )
        
        smach.StateMachine.add("fake_object",
            inspector(FakeObject),
            transitions={"success":"move_table"}
            )
        
        tablepos = poses["table_top_edge"]
        smach.StateMachine.add("move_table",
            inspector(NavigateTo, x=tablepos[0], y=tablepos[1], theta=tablepos[2]),
            transitions={"success":"putdown",
                         "failure":"failure"}
            )
        
        smach.StateMachine.add("putdown",
            inspector(PlaceDownFreeSpace),
            transitions={"success":"success",
                         "failure":"failure",
                         "still_holding":"failure"}
            )
    return sm
    
