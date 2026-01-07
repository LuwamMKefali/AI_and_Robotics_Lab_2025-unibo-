"""
============== UniBo: AI and Robotics 2025 ==============
Base code: top level execution loop
To be customized in an incremental way for the different labs

(c) 2024-2025 Alessandro Saffiotti
"""
from time import perf_counter, sleep
import robot_gwy, observer, controller


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

    def start (self, maxsteps = 0, behavior = 'Wander', behavior_params = None):
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
        self.ctr.set_behavior(bname = behavior, bparam = behavior_params)
        while not robot_gwy.is_ready():                     # wait until robot is ready
            pass
        self.run(maxsteps)                                  # run the main loop
        robot_gwy.shutdown_robot()                          # shut down robot

    def run (self, maxsteps = 0, threshold = 0.7):
        """
        Run this instance of the Top Level loop until completion
        Return True for successful execution, False for failure
        together with the degree of achievement of the running behavior
        """
        if self.debug > 0:
            print("Top level loop started")
        nsteps = 0
        done = 0.0
        while True:
            if not robot_gwy.robot_alive():                 # ROS was killed
                if self.debug > 0:
                    print("ROS killed: exiting")
                break

            # execute control pipeline
            result, done = self.step()

            # and check the results
            if result == False:                             # action failure
                if self.debug > 0:
                    print("Execution failed")
                break
            if done > threshold:                            # degree of achievement above acceptance
                if self.debug > 0:
                    print("Execution completed")
                return True, done                           # we are done, return True
            nsteps += 1
            if maxsteps > 0 and nsteps > maxsteps:          # timeout
                if self.debug > 0:
                    print("Max number of steps reached:", nsteps, ">", maxsteps, ", exiting")
                break
        return False, 0.0                                   # something wrong, return False

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

        # Read exteroceptive sensors (sonars)
        sdata = robot_gwy.get_sonar_data()
        if self.debug > 1:
            print("sonars =", " ".join(["{:.2f}".format(s[1]) for s in sdata]))

        # Compute control values
        state = {                                           # state passed to the controller
            "mypose": self.mypose,
            "sdata": sdata,
        } 
        done = self.ctr.run(state, self.debug)              # decide controls (robot's vels)
        vlin = self.ctr.get_vlin()                          # retrieve computed controls (vlin)
        vrot = self.ctr.get_vrot()                          # retrieve computed controls (vrot)
        robot_gwy.set_vel_values(vlin, vrot)                # send controls (comment this out to use kbd teleop)

        # sync on cycle time
        self.clock.sync(self.tcycle)
        if self.debug == -1:
            print("Cycle = {:.3f}".format(self.clock.elapsed()))
        self.clock.reset()
        return True, done                                   # return False to exit the loop

    def print_pose (self):
        print("Pose = ({:.2f}, {:.2f}, {:.2f})".format(self.mypose[0], self.mypose[1], self.mypose[2]))

