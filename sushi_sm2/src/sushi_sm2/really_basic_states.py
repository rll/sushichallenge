"""
Author: Lorenzo Riano <lorenzo.riano@gmail.com>
"""
PKG = "sushi_sm2"
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import pr2_python
import yaml
from collections import defaultdict
from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_python import transform_listener

DIR = roslib.packages.get_pkg_dir(PKG, required=True) + "/config/"
stream = file(DIR+"poses.yaml")
poses = yaml.load(stream)

class WorldState:
    def __init__(self):
        self.items = defaultdict(list)
        self.pickup_next = None #PickupGoal
    def add_item(self, graspable):
        """
        graspable: a PickupGoal instance
        """
        if graspable.label == "graspable":
            rospy.logwarn("Skipping a graspable")
            #return
        location = graspable.location
        self.items[location].append(graspable)

        rospy.loginfo("Adding label %s, to location: %s", graspable.label,
                location)

    def clear(self):
        self.items = defaultdict(list)
        self.pickup_next = None #PickupGoal

class PickableObject:
    def __init__(self, graspable, location):
        pose = graspable.object_pose_stamped
        self.mappose = transform_listener.transform_pose_stamped("/map",pose)
        self.graspable = graspable
        self.label = graspable.label
        self.location = location
        self.arm_name = "right_arm"

class NavigateTo(smach.State):
    """Navigate to a desired position.

    Constructor: NavigateTo(base_mover, x,y,theta)
    world_state: a WorldState instance
    base_mover is a pr2_python.base.Base instance
    x,y,theta are in the /map frame of reference (see pr2_python)
    """
    def __init__(self, world_state, base_mover, x, y, theta):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.x = x
        self.y = y
        self.theta = theta
        self.base = base_mover
        self.world_state = world_state

    def execute(self, userdata):
        pos = (self.x, self.y, self.theta)
        rospy.loginfo("%s: going to %s", self.__class__.__name__,
                     pos)
        try:
            self.base.move_to(pos[0], pos[1], pos[2])
            return "success"
        except:
            return "failure"

class Detect(smach.State):
    """Detect objects and add them to the shared userdata
    
    Constructor: Detect(world_state, detector, head_point)
    world_state: a WorldState instance
    detector: pr2_tasks TablewareDetection
    head_point: a PointStamped to move the head to. None if no head movement.
    """

    def __init__(self, world_state, location, detector, head_point = None):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.detector = detector
        self.head_point = head_point
        self.world_state = world_state
        self.location = location

    def execute(self, userdata):
        det = self.detector.detect_objects(point_head_at=self.head_point)
        if not det.pickup_goals:
            rospy.loginfo('Nothing to pick up!')
            return "failure"
        for goal in det.pickup_goals:
            obj = PickableObject(goal, self.location)
            self.world_state.add_item(obj)

        rospy.loginfo("World has now: %s", self.world_state.items)
        return "success"

class PickUp(smach.State):
    def __init__(self, world, tasks):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.world = world
        self.tasks = tasks

    def execute(self, userdata):
        goal = self.world.pickup_next
        pose = (goal.mappose.pose.position.x,
                goal.mappose.pose.position.y,
                goal.mappose.pose.position.z
               )
        graspable = goal.graspable
        graspable.arm_name = goal.arm_name
        try:
            self.tasks.go_and_pickup(goal.arm_name,pose,goal.label)
        except:
            return "failure"
        
        self.world.items[goal.location].remove(goal)
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
    def __init__(self, world, base):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.world = world
        self.base = base
    
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
                    try_hard = True, group="right_arm")
        except:
            raise
            #return "failure"
        return "success"

class MoveToReachable(smach.State):
    def __init__(self, world, base, pos):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.world = world
        self.base = base
        self.pos = pos
    
    def execute(self, userdata):
        x = self.pos[0]
        y = self.pos[1]
        z = self.pos[2]
        rospy.loginfo("Trying to go to %s", (x,y,z))
        try:
            self.base.move_manipulable_pose(x, y, z, 
                    try_hard = True, group="right_arm")
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
    def __init__(self, world, placer, pos):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.world = world
        self.placer = placer
        self.pos = pos

    def execute(self, userdata):
        try:
            self.placer.place(self.pos[0], self.pos[1], self.pos[2])
        except:
            return "failure"
        return "success"

class Resetter(smach.State):
    def __init__(self, interface):
        smach.State.__init__(self, outcomes = ["success"])
        self.interface = interface
    def execute(self, userdata):
        self.interface.reset()
        psi = get_planning_scene_interface()
        psi.reset()
        return "success"

