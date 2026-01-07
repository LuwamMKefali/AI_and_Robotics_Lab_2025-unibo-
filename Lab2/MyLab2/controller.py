"""
============== UniBo: AI and Robotics 2025 ==============
Base code: skeleton controller, performs the action decision parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024-2025 Alessandro Saffiotti
"""

from fcontrol import Behavior
from fcontrol import ramp_up, ramp_down, triangle, trapezoid
from fcontrol import global_to_local, local_to_global
from math import atan2, degrees, sqrt


class Still (Behavior):
    """
    Just stay still. This is the default behavior.
    """
    def __init__(self, noparam):
        super().__init__()
    
    def update_state(self, state):
        """
        No local state variables to set
        """

    def setup(self):
        """
        Predicates, linguistic variables and rules of our fuzzy behavior
        """
        self.fpreds = {
            # No fuzzy predicates used in the LHS of rules
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.5, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':30, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-30}, 'Vrot')
        }

        self.frules = {
            'NoTurn'  : ("True", 'Turn', 'None'),
            'NoGo'    : ("True", 'Move', 'None')
        }

        # The degree of achievement is given by the built-in predicate 'False' to run forever
        self.fgoal = "False"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()


class GoToTarget (Behavior):
    """
    Navigate to a target object at (x,y,th,radius), where 'x,y' is
    the object's center in global frame, and 'radius' is its size
    The behavior stops at about 'radius' mt from the object's center
    The object's orientation th is ignored
    It assumes that there are no obstacles on the way
    """
    def __init__(self, target = (0.0, 0.0, 0.0, 0.0)):
        self.tpoint = [target[0], target[1], 0.0]   # target center point in global coordinates
        self.tlocal = [0, 0, 0]                     # target center point in robot's coordinates
        print("Target: " , target)
        self.radius = target[3]                     # distance to stop from target's centerpoint
        super().__init__()
    
    def update_state(self, state):
        """
        Set the local state variables phi and rho
        Use the 'mypose' estimate, passed as part of the global state
        """

        # 'phi' and 'rho' are radial coordinates to target
        # 'phi' is in degrees, 'rho' is in meters
        mypose = state['mypose']
        global_to_local(self.tpoint, mypose, self.tlocal)
        xt = self.tlocal[0]
        yt = self.tlocal[1]
        self.state['phi'] = degrees(atan2(yt, xt))
        self.state['rho'] = sqrt(xt * xt + yt * yt)

    def setup(self):
        """
        Predicates, linguistic variables and rules of our fuzzy behavior
        """
        robot_radius = 0.5
        stop_dist = self.radius + robot_radius

        self.fpreds = {
            # Fuzzy predicates used in the LHS of rules
            'TargetLeft'  : (ramp_up(30.0, 60.0), 'phi'),
            'TargetRight' : (ramp_down(-60.0, -30.0), 'phi'),
            'TargetAhead' : (triangle(-30.0, 0.0, 30.0), 'phi'),
            'TargetHere'  : (ramp_down(stop_dist, stop_dist+1.0), 'rho'),
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.5, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':30, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-30}, 'Vrot')
        }

        self.frules = {
            'ToLeft'  : ("TargetLeft AND NOT(TargetHere)", 'Turn', 'Left'),
            'ToRight' : ("TargetRight AND NOT(TargetHere)", 'Turn', 'Right'),
            'Straight' : ("TargetAhead", 'Turn', 'None'),
            #'Far'     : ("TargetAhead AND NOT(TargetHere)", 'Move', 'Fast'),
            #'Stop'    : ("TargetHere OR NOT(TargetAhead)", 'Move', 'None')
            'GoFast'  : ("TargetAhead AND NOT(TargetHere)", 'Move', 'Fast'),
            'GoSlow'  : ("NOT(TargetAhead) AND(TargetHere)", 'Move', 'Slow'),

            'Stop'    : ("TargetHere", 'Move', 'None')
        }

        # The degree of achievement is given by the predicate 'TargetHere'
        self.fgoal = "TargetHere"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()


class Wander (Behavior):
    def __init__(self, noparam):
        super().__init__()
    
    def update_state(self, state):
        """
        Set the local state variables for clearance (left, right, front, overall)
        Uses the 'sdata' sonar ranges, passed as part of the global state
        """
        ranges = [s[1] for s in state['sdata']]
        self.state['lft_clr'] = min(ranges[1], ranges[2])
        self.state['rgt_clr'] = min(ranges[10], ranges[11])
        self.state['frt_clr'] = ranges[0]
        self.state['all_clr'] = min(ranges)

    def setup(self):
        """
        Predicates, linguistic variables and rules of our fuzzy behavior
        """
        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'ObstacleLeft'  : (ramp_down(0.0, 1.0), 'lft_clr'),
            'ObstacleRight' : (ramp_down(0.0, 1.0), 'rgt_clr'),
            'ObstacleAhead' : (ramp_down(0.0, 1.0), 'frt_clr'),
            # Fuzzy predicate do decide between avoid and go
            'Danger'        : (ramp_down(0.0, 1.0), 'all_clr')
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.5, 'Slow':0.1, 'None':0, 'Back':-0.5}, 'Vlin'),
            'Turn' : ({'Left':30, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-30}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'GoLeft'   : ("ObstacleRight AND NOT(ObstacleLeft)", 'Turn', 'Left'),
            'GoRight'  : ("ObstacleLeft AND NOT(ObstacleRight)", 'Turn', 'Right'),
            'Tunnel'   : ("ObstacleRight AND ObstacleLeft", 'Turn', 'None'),
            'Front'    : ("ObstacleAhead", 'Move', 'Back'),
            'Slow'     : ("Danger", 'Move', 'Slow'),
            'Fast'     : ("NOT(Danger)", 'Move', 'Fast')
        }

        # The degree of achievement is set to false: this behavior never ends
        self.fgoal = "False"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()


class Controller ():
    def __init__(self, robot_pars):
        self.behavior = None        # top-level fuzzy behavior run by the controller
        self.achieved = 0.0         # level of achievement of current behavior
        self.vlin = 0.0             # current value for vlin control variable
        self.vrot = 0.0             # current value for vrot control variable

    def set_behavior (self, bname = None, bparam = None):
        """
        Initialize the controller, setting the behavior to be executed
        Return True if the inizialization is successful
        """
        if bname:
            self.behavior = globals()[bname](bparam)
        return True

    def run (self, state, debug):
        """
        Action decision. Compute control values (vlin, vrot) given the current robot's pose
        by running the current behavior; return the level of achievement of that behavior
        """
        self.achieved = self.behavior.run(state, debug)
        self.vlin = self.behavior.get_vlin()
        self.vrot = self.behavior.get_vrot()
        if debug > 1:
            print('Goal achievement: {:.2f}'.format(self.achieved))
        if debug > 1:
            print('(vlin, vrot) = ({:.2f}, {:.2f})'.format(self.vlin, degrees(self.vrot))) 
        return self.achieved

    def get_vlin (self):
        return self.vlin
    
    def get_vrot (self):
        return self.vrot
    
