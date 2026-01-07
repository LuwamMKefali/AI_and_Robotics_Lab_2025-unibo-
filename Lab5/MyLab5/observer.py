"""
============== UniBo: AI and Robotics 2025 ==============
Base code: dummy observer, performs the state estimation parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024-2025 Alessandro Saffiotti
"""
from math import sin, cos

class Observer:
    """
    Initialize the observer, using the given robot's parameters
    """ 
    def __init__ (self, robot_pars):
        self.WHEEL_RADIUS = robot_pars['wheel_radius']
        self.WHEEL_AXIS = robot_pars['wheel_axis']
        self.gx = 0.0         # current estimate of robot's global position
        self.gy = 0.0
        self.th = 0.0

    # set initial pose
    def init_pose (self, pose = None):
        """
        Set initial pose of the robot
        """
        if pose != None:
            self.gx = pose[0]
            self.gy = pose[1]
            self.th = pose[2]

    def update_pose (self, wl, wr):
        """
        Incremental position estimation: update the current pose (x, y, th) of the robot
        taking into account the newly read positions (rotations) of the left and right wheels
        Returns the new robot's pose, as a triple (x, y, th)
        """
        return (self.gx, self.gy, self.th)

