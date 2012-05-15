def rope_dist(d0):
    return l2dist(align_rope(rope0, rope1), rope1)


class State(object):
    @classmethod
    def calc_actions(state, d):
        abstract
    @classmethod
    def calc_score(state,d):
        return rope_dist(d["rope"], state.exemplar["rope"])

class Arch(State):
    exemplar = EXEMPLARS["arch"]
    @classmethod
    def calc_actions(state, d):
        grab_pose = align_rope(TRAINING["arch_grab"], d)["left_pose"]
        release_pose = align_rope(TRAINING["arch_grab"], d)["left_pose"]
        grab_and_move = GrabAndMove(grab_pose, release_pose, "left")
        
        adjustments = calc_adjustment_actions(d["rope"], TRAINING["arch"]["rope"])
                
        return grab_and_move + adjustments

       
class LoopDiagRightEnd(State):
    exemplar = EXEMPLARS["loop_diag_right_end"]
    @classmethod
    def calc_actions(state,d):
        grab_pose = align_rope(EXEMPLARS["straighten_right_end_grab"], d)["right_pose"]
        release_pose = align_rope(EXEMPLARS["straighten_right_end_release"], d)["right_pose"]
        grab_and_move = GrabAndMove(grab_pose, release_pose, "right")
        
class LoopVertRightEnd(State):
    exemplar = EXEMPLARS["loop_vert_right_end"]
    @classmethod
    def calc_actions(state, d):
        grab_pose = 
        # rotate one end, drag the whole thing, move end
        
class StrangledGhost(State):
    exemplar = EXEMPLARS["strangled_ghost"]

class Action(object):
    def __init__(self):
        abstract
    def run(self):
        abstract

class GrabAndMove(Action):
    def __init__(self, grab_pose, release_pose):
        pass
    def run(self):
        abstract
    
        
class HumanChooser(Executive):
    states = []
    def run(self):
        while True:
            current_state = calc_most_likely_state(states)
            candidate_actions = current_state.calc_actions()
            human_picks_action(candidate_actions)
            if current_state.terminal():
                break
            
        
    