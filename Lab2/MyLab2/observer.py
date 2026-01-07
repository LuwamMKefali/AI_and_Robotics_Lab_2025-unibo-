"""
============== UniBo: AI and Robotics 2025 ==============
Base code: dummy observer, performs the state estimation parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024-2025 Alessandro Saffiotti
"""
from math import sin, cos, pi

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

        self.prev_wl = None   # previous wheel angles, used to compute increments
        self.prev_wr = None

    # set initial pose
    def init_pose (self, pose = None):
        """
        Set initial pose of the robot
        """
        if pose != None:
            self.gx = pose[0]
            self.gy = pose[1]
            self.th = pose[2]

        self.prev_wl = None
        self.prev_wr = None

    def update_pose (self, wl, wr):
        """
        Incremental position estimation: update the current pose (x, y, th) of the robot
        taking into account the newly read positions (rotations) of the left and right wheels
        Returns the new robot's pose, as a triple (x, y, th)
        """

        if self.prev_wl == None or self.prev_wr == None:
            self.prev_wl = wl
            self.prev_wr = wr
            return (self.gx, self.gy, self.th)

        d_wl = wl - self.prev_wl
        d_wr = wr - self.prev_wr

        self.prev_wl = wl
        self.prev_wr = wr

        dl = d_wl * self.WHEEL_RADIUS
        dr = d_wr * self.WHEEL_RADIUS

        dc = (dl + dr) / 2.0
        dth = (dr - dl) / self.WHEEL_AXIS

        self.gx = self.gx + dc * cos(self.th + dth / 2.0)
        self.gy = self.gy + dc * sin(self.th + dth / 2.0)
        self.th = self.th + dth

        while self.th > pi:
            self.th = self.th - 2.0 * pi
        while self.th < -pi:
            self.th = self.th + 2.0 * pi

        return (self.gx, self.gy, self.th)
