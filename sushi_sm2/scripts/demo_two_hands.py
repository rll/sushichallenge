#!/usr/bin/env python
"""
Author: Lorenzo Riano <lorenzo.riano@gmail.com>
"""
import roslib; 
roslib.load_manifest("sushi_sm2")
import rospy
import smach
import smach_ros
from pr2_python import base
from sushi_sm2 import two_arms_states 
from pr2_tasks.tableware_detection import TablewareDetection
from pr2_tasks.pickplace import PickPlace
from pr2_python.world_interface import WorldInterface
from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_tasks.tasks import Tasks
from sensing_placer.place import SensingPlacer

def create_pickup_sm(world, tasks, location):
    sm = smach.StateMachine(outcomes=["success",
                                      "failure",
                                      ])
    with sm:
        smach.StateMachine.add("choose",
                two_arms_states.ChooseItem(world, location),
                transitions = {"success":"pickup",
                    "failure":"success"}
                )
        smach.StateMachine.add("pickup", 
                    two_arms_states.PickUp(world, tasks),
                    transitions = {"success":"choose",
                                   "no_free_arm":"success",
                                   "failure":"failure"
                                  }
                    )
    return sm

def create_detect_sm(world, base_mover, detector):
    sm = smach.StateMachine(outcomes=["success",
                                      "failure"])
    poses = two_arms_states.poses

    with sm:
        #TABLE
        pos = poses["table_top_edge"]
        smach.StateMachine.add("move_top_edge",
                two_arms_states.NavigateTo(world, base_mover,
                    pos[0],pos[1],pos[2]
                    ),
                transitions = {"success":"detect_table",
                               "failure":"failure"
                              }
                )
         
        smach.StateMachine.add("detect_table",
                two_arms_states.Detect(world, "table", detector),
                transitions = {"success":"move_shelf",
                               "failure":"failure"}
                )

        #SHELF
        pos = poses["shelf"]
        smach.StateMachine.add("move_shelf",
                two_arms_states.NavigateTo(world, base_mover,
                    pos[0],pos[1],pos[2]
                    ),
                transitions = {"success":"detect_shelf",
                               "failure":"failure"
                              }
                )
         
        smach.StateMachine.add("detect_shelf",
                two_arms_states.Detect(world, "shelf", detector),
                transitions = {"success":"success",
                               "failure":"failure"}
                )
    return sm

def clean_table_sm(world, tasks, base_mover, placer_l, placer_r, interface):
    sm = smach.StateMachine(outcomes=["success",
                                      "failure"])
    poses = two_arms_states.poses
    location = "table"
    pos = poses["dirty_table_putdown"]
    
    with sm:
        smach.StateMachine.add("choose",
                two_arms_states.ChooseItem(world, location),
                transitions = {"success":"pickup",
                    "failure":"success"}
                )
        
        pickup_sm = create_pickup_sm(world, tasks, location)
        
        smach.StateMachine.add("pickup",
                pickup_sm,
                transitions = {"success":"move_dirty_table",
                              "failure":"reset2",
                              }
            )
        smach.StateMachine.add("reset2",
                two_arms_states.Resetter(interface),
                transitions = {"success":"pickup"}
                )
        
        smach.StateMachine.add("move_dirty_table",
                two_arms_states.MoveToReachable(world, base_mover, pos),
                transitions = {"success":"putdown",
                               "failure":"failure"
                              }
        )

        smach.StateMachine.add("move_arms_side",
                two_arms_states.MoveArmsToSide(world, tasks),
                transitions = {"success":"putdown",
                               "failure":"putdown"
                              }
        )
        
        smach.StateMachine.add("putdown",
                two_arms_states.PlaceDown(world, placer_l, placer_r, pos),
                transitions = {"success":"reset",
                               "failure":"putdown",
                               "still_holding":"move_arms_side"
                              }
                )
        
        smach.StateMachine.add("reset",
                two_arms_states.Resetter(interface),
                transitions = {"success":"choose"}
                )
                               
    return sm

def setup_table_sm(world, tasks, base_mover, placer_l, placer_r, interface):
    sm = smach.StateMachine(outcomes=["success",
                                      "failure"])
    poses = two_arms_states.poses
    pos = poses["table_putdown_top"]
    location = "shelf"
    
    with sm:
        smach.StateMachine.add("choose",
                two_arms_states.ChooseItem(world, location),
                transitions = {"success":"pickup",
                    "failure":"success"}
                )
        
        pickup_sm = create_pickup_sm(world, tasks, location)
        
        smach.StateMachine.add("pickup",
                pickup_sm,
                transitions = {"success":"move_dirty_table",
                              "failure":"reset2",
                              }
            )
        smach.StateMachine.add("reset2",
                two_arms_states.Resetter(interface),
                transitions = {"success":"pickup"}
                )
        
        smach.StateMachine.add("move_dirty_table",
                two_arms_states.MoveToReachable(world, base_mover, pos),
                transitions = {"success":"putdown",
                               "failure":"failure"
                              }
        )

        smach.StateMachine.add("move_arms_side",
                two_arms_states.MoveArmsToSide(world, tasks),
                transitions = {"success":"putdown",
                               "failure":"putdown"
                              }
        )
        
        smach.StateMachine.add("putdown",
                two_arms_states.PlaceDown(world, placer_l, placer_r, pos),
                transitions = {"success":"reset",
                               "failure":"putdown",
                               "still_holding":"move_arms_side"
                              }
                )
        
        smach.StateMachine.add("reset",
                two_arms_states.Resetter(interface),
                transitions = {"success":"choose"}
                )
       
    return sm
     
def main():
    rospy.init_node("touring", anonymous=True)

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

    interface.reset()
    psi = get_planning_scene_interface()
    psi.reset()
    tasks.move_arms_to_side()

    rospy.loginfo("Creating State Machine")

    with sm:
        detect = create_detect_sm(world, base_mover, detector)
        smach.StateMachine.add("detect", detect,
                transitions = {"success":"clean_table",
                               "failure":"failure"}
                )

        clean_table = clean_table_sm(world, tasks, base_mover, placer_l, placer_r, interface)
        smach.StateMachine.add("clean_table", clean_table,
                transitions = {"success":"setup_table",
                               "failure":"failure"
                               }
                )
        
        setup_table = setup_table_sm(world, tasks, base_mover, placer_l, placer_r, interface)
        smach.StateMachine.add("setup_table", setup_table,
                transitions = {"success":"success",
                               "failure":"failure"
                               }
                )


    outcome = sm.execute()
    rospy.loginfo("Outcome: %s", outcome)


if __name__ == "__main__":
    main()
        
