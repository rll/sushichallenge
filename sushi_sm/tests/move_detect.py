#!/usr/bin/env python
"""
Author: Lorenzo Riano <lorenzo.riano@gmail.com>
"""
import roslib; 
roslib.load_manifest("sushi_sm")
import rospy
import smach
import smach_ros
from pr2_python import base
from sushi_sm import basic_states
from pr2_tasks.tableware_detection import TablewareDetection
from pr2_tasks.pickplace import PickPlace
from pr2_python.world_interface import WorldInterface
from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_tasks.tasks import Tasks
from sensing_placer.place import SensingPlacer

def main():
    rospy.init_node("touring", anonymous=True)

    sm = smach.StateMachine(outcomes=["success",
                                      "failure"])
    base_mover = base.Base()
    detector = TablewareDetection()
    pickplace = PickPlace()
    pickplace.min_open_gripper_pos = 0.0014
    poses = basic_states.poses
    world = basic_states.WorldState()
    interface = WorldInterface()
    tasks = Tasks()
    placer = SensingPlacer('right_arm')

    interface.reset()
    psi = get_planning_scene_interface()
    psi.reset()
    tasks.move_arms_to_side()

    rospy.loginfo("Creating State Machine")
    nav_states = {}
    for name, pos in poses.iteritems():
        nav_states[name] = basic_states.NavigateTo(
        world, base_mover, pos[0], pos[1], pos[2])

    detect_state = basic_states.Detect(world, detector)
    
    top_pos_place_down = (-2.15, .5, .98)
    place_down_top = basic_states.PlaceDown(world, placer,
            top_pos_place_down)

    T = {"failure":"failure"}
    with sm:
        T["success"] = "detect"
        T["failure"] = "failure"
        
        smach.StateMachine.add("move_shelf", nav_states["shelf"],
                transitions=T.copy())
        T["success"] = "choose_bowl"
        T["failure"] = "failure"


        smach.StateMachine.add("detect", detect_state,
                transitions=T.copy()
                )
        T["success"] = "move_to_pickable"
        T["failure"] = "failure"
        
        smach.StateMachine.add("choose_bowl", 
                basic_states.ChooseItem(world, "bowl"),
                transitions=T.copy()
                )
        T["success"] = "pickup"
        T["failure"] = "pickup"
        
        smach.StateMachine.add("move_to_pickable", 
                basic_states.MoveToPickable(world, base_mover ),
                transitions=T.copy()
                )
        T["success"] = "move_arms_to_side"
        T["failure"] = "move_arms_to_side_after_pickup"
        
        smach.StateMachine.add("pickup", 
                basic_states.PickUp(world, pickplace, interface),
                transitions=T.copy()
                )
        T["success"] = "detect"
        T["failure"] = "failure"
        
        smach.StateMachine.add("move_arms_to_side_after_pickup", 
                basic_states.MoveArmsToSide(world, tasks),
                transitions=T.copy()
                )
        T["success"] = "move_top_table"
        T["failure"] = "failure"
        
        smach.StateMachine.add("move_arms_to_side", 
                basic_states.MoveArmsToSide(world, tasks),
                transitions=T.copy()
                )
        T["success"] = "approach_top_table"
        T["failure"] = "failure"
        
        smach.StateMachine.add("move_top_table", 
                nav_states["table_top_edge"],
                transitions=T.copy())
        T["success"] = "place_down_top"
        T["failure"] = "failure"
        
        smach.StateMachine.add("approach_top_table", 
                basic_states.MoveToReachable(world, base_mover, top_pos_place_down),
                transitions=T.copy())
        T["success"] = "success"
        T["failure"] = "failure"
        
        smach.StateMachine.add("place_down_top", 
                place_down_top,
                transitions=T.copy())

    outcome = sm.execute()
    rospy.loginfo("Outcome: %s", outcome)


if __name__ == "__main__":
    main()
        
