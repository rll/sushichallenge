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
from sushi_sm import two_arms_states 
from pr2_tasks.tableware_detection import TablewareDetection
from pr2_tasks.pickplace import PickPlace
from pr2_python.world_interface import WorldInterface
from pr2_python.head import Head
from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_tasks.tasks import Tasks
from sensing_placer.place import SensingPlacer

from sushi_sm.states_inspector import StateInspector

from sushi_sm.two_arms_states import create_pickup_sm
from sushi_sm.two_arms_states import create_detect_sm
from sushi_sm.two_arms_states import create_clean_table_sm
from sushi_sm.two_arms_states import create_detect_shelves_sm
from sushi_sm.two_arms_states import create_setup_table_sm
from sushi_sm.two_arms_states import create_spinning_table_clear_sm
     
     
def main():
    rospy.init_node("demo_berkeley", anonymous=True)

    sm = smach.StateMachine(outcomes=["success",
                                      "failure"])
    base_mover = base.Base()
    detector = TablewareDetection()
    pickplace = PickPlace()
    pickplace.min_open_gripper_pos = 0.0014
    world = two_arms_states.WorldState()
    interface = WorldInterface()
    tasks = Tasks()
    placer_r = SensingPlacer('right_arm')
    placer_l = SensingPlacer('left_arm')
    head = Head()

    rospy.loginfo("Creating State Machine")
    
    inspector = StateInspector(two_arms_states)
    inspector.world = world
    inspector.tasks = tasks
    inspector.base_mover = base_mover
    inspector.interface = interface
    inspector.detector = detector
    inspector.placer_r = placer_r
    inspector.placer_l = placer_l
    inspector.pickplace = pickplace
    inspector.head = head
    

    with sm:
        
        smach.StateMachine.add("reset",
                               inspector(two_arms_states.Resetter),
                               transitions={"success":"move_arms_side"})
        
        smach.StateMachine.add("move_arms_side",
                               inspector(two_arms_states.MoveArmsToSide),
                               transitions={"success":"spinning_table_pickup"})

        smach.StateMachine.add("spinning_table_pickup", create_spinning_table_clear_sm(),
                               transitions = {"success":"success",
                                              "failure":"failure"}
                               )
#        clean_table = create_clean_table_sm(inspector)
#        smach.StateMachine.add("clean_table", clean_table,
#                transitions = {"success":"success",
#                               "failure":"failure"
#                               }
#                )
        
#        setup_table = create_setup_table_sm(inspector)
#        smach.StateMachine.add("setup_table", setup_table,
#                transitions = {"success":"success",
#                               "failure":"failure"
#                               }
#                )

    
    sis = smach_ros.IntrospectionServer('sushi_sm', sm, '/SM_ROOT')
    sis.start()
    
    try:
        outcome = sm.execute()
        rospy.loginfo("Outcome: %s", outcome)
        sis.stop()
    except Exception:
        sm.call_termination_cbs(None,None)

if __name__ == "__main__":
    main()
        
