"""
Author: Lorenzo Riano <lorenzo.riano@gmail.com>
"""
PKG = "sushi_sm"
import rospy
from collections import defaultdict
from pr2_python import transform_listener


class SquarenessFilter(object):
    def __init__(self, find_box, threshold):
        self.find_box = find_box
        self.threshold = threshold
    
    def __call__(self, item):
        """
        item: A PickableObject instance
        returns: True if the squareness of item is greater than self.threshold,
                 False otherwise.
        """
        graspable = item.graspable
        pointcloud = graspable.target.cluster
        box = self.find_box(pointcloud)
        dims = box.box_dims
        squareness = min(dims.x, dims.y) / max(dims.x, dims.y)
        rospy.loginfo("Object %s, squareness: %f", item.label, squareness)
        if squareness >= self.threshold:
            return True
        else:
            return False
        

class WorldState(object):
    def __init__(self, find_box):
        rospy.loginfo("Waiting for find_cluster_bounding_box2 service")
        self.find_box = find_box 
        self.items = defaultdict(list)
        self.list_of_item_types = defaultdict(list) 
        self.pickup_next = None #PickupGoal
        self.free_arms = ["right_arm", "left_arm"]
        self.dining_setup = {"top":False, "left":False}
        self.object_in_hands = {} #{left: obj, right:obj}
        self.filters = [SquarenessFilter(self.find_box, 0.53)]
        self.failed_to_pickup = None
        self.table_locations = ["table_top_edge",
                                "table_left_edge",
                                "table_right_edge"]
        self.round_robin_location = 0

    def get_next_round_robin_location(self):
        loc = self.table_locations[self.round_robin_location]
        self.round_robin_location = (self.round_robin_location +1) % len(self.table_locations)
        return loc

    def add_item(self, graspable):
        """
        graspable: a PickableObject instance
        """        
        if graspable.label == "graspable" or graspable.label == "plate" :
            pointcloud = graspable.graspable.target.cluster
            box = self.find_box(pointcloud)
            dims = box.box_dims
            if dims.z <= 0.02:
                graspable.label = "plate"
            res = all(f(graspable) for f in self.filters)
            if not res:
                rospy.loginfo("Skipping filtered %s", graspable.label)
                return

        location = graspable.location
        self.items[location].append(graspable)
        self.list_of_item_types[graspable.label].append(graspable)

        rospy.loginfo("Adding label %s, to location: %s", graspable.label,
                location)

    def remove_item(self, graspable):
        """
        graspable: a PickableObject instance
        """
        rospy.loginfo("Trying to remove %s", graspable)
        try:
            self.items[graspable.location].remove(graspable)
        except ValueError:
            rospy.logwarn("It hasn't been found! The worlds has: %s", self.items)

    def clear(self):
        self.items = defaultdict(list)
        self.pickup_next = None #PickupGoal
        self.free_arms = ["right_arm", "left_arm"]
        self.dining_setup = {"top":False, "left":False}
        self.object_in_hands = {} #{left: obj, right:obj}

class PickableObject(object):
    """An object the robot can pickup.
    
    Constructor: PickableObject(graspable, location)
    graspable: a pr2_python.pickplace_definitions.PickupGoal instance
    location: a string that identifies where the object is
    """
    def __init__(self, graspable, location):        
        self.pose_stamped = graspable.object_pose_stamped
        self.mappose = transform_listener.transform_pose_stamped("/map",
                                                                 self.pose_stamped)
        self.graspable = graspable
        self.label = graspable.label
        self.location = location
        self.arm_name = "right_arm"