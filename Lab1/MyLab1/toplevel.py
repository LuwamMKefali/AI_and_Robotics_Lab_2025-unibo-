"""
============== UniBo: AI and Robotics 2025 ==============
Base code: top level execution loop
To be customized in an incremental way for the different labs

(c) 2024-2025 Alessandro Saffiotti
"""
from time import perf_counter, sleep
import robot_gwy, observer, controller
import math
import os


class Timer:
    """
    Timing functions for synchronizing the control loop
    """
    def __init__(self):
        self.last = perf_counter()

    def reset(self):
        self.last = perf_counter()

    def elapsed(self):
        """Retun the time elapsed since last sync"""
        return perf_counter() - self.last

    def sync(self, cycle):
        """Sleep until elapsed time is cycle"""
        current = perf_counter()
        elapsed = current - self.last
        if cycle > elapsed:
            sleep(cycle - elapsed)


class TopLevelLoop:
    """
    Top level execution loop
    """

    def __init__(self,
                mypose = None,      # robot's starting pose, takes from the map if None
                tcycle = 0.1,       # cycle time, in sec
                debug = 0           # debug level, use to decide what debug info to print
                ):
        self.tcycle = tcycle
        self.debug = debug
        self.mypose = mypose        # estimated robot's pose
        self.obs = None             # observer instance
        self.ctr = None             # controller instance
        self.clock = None           # timer instance
        robot_gwy.init_ros()

    def start (self, maxsteps = 0):
        """
        Start the Top Level loop, initialize all the components, and run it
        Times out with failure after 'maxsteps' steps (zero = no timeout)
        Debug level can be used to selectively print debug information
        """
        robot_params = robot_gwy.startup_robot()            # start robot and get its parameters
        self.clock = Timer()                                # create timer
        self.obs = observer.Observer(robot_params)          # create observer
        self.obs.init_pose(self.mypose)                     # give inital position to observer
        self.ctr = controller.Controller(robot_params)      # create controller
        self.logfile = None
        self.gt0 = None # initial ground truth pose

        # fixed alignment offsets between GT and odom frames (computed once)
        self.gt_offset = None

        if self.debug > 0:
            # choose log file name via environment variable to avoid mixing runs
            logname = os.environ.get("LAB1_LOGNAME", "lab1_traj_log.csv")

            # overwrite each run (no appending), so good and bad experiments do not mix
            self.logfile = open(logname, "w")
            self.logfile.write("x_est,y_est,th_est,x_gt,y_gt,th_gt,pos_err\n")

            if self.debug > 0:
                print("Logging to:", logname)

        while not robot_gwy.is_ready():                     # wait until robot is ready
            pass
        self.run(maxsteps)                                  # run the main loop
        if self.logfile != None:
            self.logfile.close()
        robot_gwy.shutdown_robot()                          # shut down robot

    def run (self, maxsteps = 0):
        """
        Run this instance of the Top Level loop until completion
        Return True for successful execution, False for failure
        """
        if self.debug > 0:
            print("Top level loop started")
        nsteps = 0
        while True:
            if not robot_gwy.robot_alive():                 # ROS was killed
                if self.debug > 0:
                    print("ROS killed: exiting")
                return False
            result = self.step()                            # execute control pipeline once
            if not result:                                  # execution failed
                if self.debug > 0:
                    print("Execution failed: exiting")
                return False
            nsteps += 1
            if maxsteps > 0 and nsteps > maxsteps:          # timeout
                if self.debug > 0:
                    print("Max number of steps reached:", nsteps, ">", maxsteps, ", exiting")
                return True

    def step (self):
        """
        The basic control pipeline: read sensors, estimate state, decide controls, send controls
        Return True for successful execution, plus the current degree of achievement
        """

        # Read proprioceptive sensors (wheel rotations)
        wl, wr = robot_gwy.get_wheel_encoders()             # read wheel encoders
        self.mypose = self.obs.update_pose(wl, wr)          # estimate state (robot's pose)
        if self.debug > 1:
            self.print_pose()
        
        gt_pose = robot_gwy.get_ground_truth_pose()

        if gt_pose != None:

            # store initial ground truth pose
            if self.gt0 == None:
                self.gt0 = gt_pose

            # align ground truth frame to odometry frame, because I was getting wacky plots
            # NOTE: compute a FIXED offset once, instead of adding mypose at every step
            if self.gt_offset == None:
                self.gt_offset = (
                    self.mypose[0] - self.gt0[0],
                    self.mypose[1] - self.gt0[1],
                    self.mypose[2] - self.gt0[2]
                )

            x_gt = gt_pose[0] + self.gt_offset[0]
            y_gt = gt_pose[1] + self.gt_offset[1]
            th_gt = gt_pose[2] + self.gt_offset[2]

            if self.logfile != None:
                ex = self.mypose[0] - x_gt
                ey = self.mypose[1] - y_gt
                pos_err = math.sqrt(ex * ex + ey * ey)

                self.logfile.write(
                    "{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}\n".format(
                        self.mypose[0], self.mypose[1], self.mypose[2],
                        x_gt, y_gt, th_gt,
                        pos_err
                    )
                )

        # Compute control values
        state = {                                           # state passed to the controller
            "mypose": self.mypose,
        } 
        self.ctr.run(state, self.debug)                     # decide controls (robot's vels)
        vlin = self.ctr.get_vlin()                          # retrieve computed controls (vlin)
        vrot = self.ctr.get_vrot()                          # retrieve computed controls (vrot)
        robot_gwy.set_vel_values(vlin, vrot)                # send controls (comment out to use kbd teleop)

        # sync on cycle time
        self.clock.sync(self.tcycle)
        if self.debug == -1:
            print("Cycle = {:.3f}".format(self.clock.elapsed()))
        self.clock.reset()
        return True                                         # return False to exit the loop

    def print_pose (self):
        print("Pose = ({:.2f}, {:.2f}, {:.2f})".format(self.mypose[0], self.mypose[1], self.mypose[2]))
