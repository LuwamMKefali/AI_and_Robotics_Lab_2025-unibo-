"""
============== UniBo: AI and Robotics 2025 ==============
Base code: top level execution loop
To be customized in an incremental way for the different labs

(c) 2024-2025 Alessandro Saffiotti
"""
from time import perf_counter, sleep
import robot_gwy, observer, controller
from htn_domain import State
from world_map import map
from pyhop import pyhop


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
                goal = [],                  # top level task, given to the planner
                mypose = (0.0, 0.0, 0.0),   # robot's starting pose
                tcycle = 0.1,               # cycle time, in sec
                debug = 0                   # debug level, use to decide what debug info to print
                ):
        self.goal = goal
        self.tcycle = tcycle
        self.debug = debug
        self.mypose = mypose                # estimated robot's pose
        self.obs = None                     # observer instance
        self.ctr = None                     # controller instance
        self.clock = None                   # timer instance
        robot_gwy.init_ros()

    def start (self, goal, maxsteps = 0):
        """
        Start the Top Level loop, initialize all the components, and run it
        Times out with failure after 'maxsteps' steps (zero = no timeout)
        Debug level can be used to selectively print debug information
        """
        robot_params = robot_gwy.startup_robot()            # start robot and get its parameters
        self.clock = Timer()                                # create timer
        self.obs = observer.Observer(robot_params)          # create observer
        self.obs.init_pose(self.mypose)                     # set inital position to observer
        self.ctr = controller.Controller(robot_params)      # create controller
        while not robot_gwy.is_ready():                     # wait until robot is ready
            pass
        self.sense_plan_act(goal, maxsteps)                 # run the main loop
        robot_gwy.shutdown_robot()

    def sense_plan_act(self, goal, maxsteps=0):
        """
        The outer "SPA" loop
        """
        state = State()
        self.get_state(state)                           # S = Sense
        if self.debug > 0:
            print(f"Planner called from initial state: {state}")

        plan = pyhop(state, goal, verbose=1)            # P = Plan
        if self.debug > 0:
            print(f"Planner returned plan: {plan}")
        if plan:
            result = self.execute_plan(plan, maxsteps)  # A = Act
            if self.debug > 0:
                if result:
                    print("Plan execution completed!")
                else:
                    print("Plan execution failed!")
            return result
        else:
            if self.debug > 0:
                print("No plan found!")
        return None

    def get_state (self, state):
        """
        The "Sense" part of the SPA loop
        Fill the planner state with the current values, taken from the map
        or from the robot's sensors
        NOTE: needs to be updated to consider your specific "State"
        """
        # first, set the static part of the state, taken from the map
        for (door, room1, room2) in map.find_doors():       # doors' connectivity
            state.connects[door] = (room1, room2)
        for room, contents in map.topology.items():         # objects' room, except for doors
            for object in contents:
                if object in state.connects:
                    continue
                state.room[object] = room

        # second, set the dynamic part of the state, updated through the robot's sensor
        state.pos["me"] = map.find_location(self.mypose)
        state.room["me"] = map.find_room(self.mypose)
        for box in ["box1", "box2", "box3"]:
            boxpos = robot_gwy.get_box_position(box)
            state.pos[box] = boxpos
            state.room[box] = map.find_room(boxpos)
        return state

    def execute_plan (self, plan, maxsteps):
        """
        The "Act" part of the SPA loop
        Pop each action in the plan in sequence, and execute it
        Return True for successful plan execution, False for failure
        """
        if self.debug > 0:
            print("Executing plan")
        for action in plan:
            result = self.execute_action(action, maxsteps)
            if result == False:
                break
        return result

    def execute_action (self, action, maxsteps = 0, threshold = 0.7):
        """
        Run the control pipeline for 'action' until completion, that is until
        its degree of achievement is greter than threshold (or failure, or timeout)
        Return True for successful execution, False for failure
        """
        if self.debug > 0:
            print("Executing action:", action)
        nsteps = 0

        # set the behavior to be used, and its parameter
        behavior = action[0]
        argument = action[1]
        if behavior in ['Open', 'Close', 'PickUp', 'PutDown']:
            param = argument                                    # use symbolic parameter
            print("Behavior", behavior, "not implemented: skipped")
            return True
        elif argument in ["box1", "box2", "box3"]:
            param = robot_gwy.get_box_position(argument)        # get metric parameter from server
        else:
            param = map.locations[argument]                     # get metric parameter from map
        self.ctr.set_behavior(bname=behavior, bparam=param)

        # now run the control pipeline for that behavior
        nsteps = 0
        done = 0.0
        while True:
            if not robot_gwy.robot_alive():             # ROS was killed
                if self.debug > 0:
                    print("ROS killed: exiting")
                break
            result, done = self.step()                  # execute control pipeline
            nsteps += 1
            if result == False:                         # action failure
                if self.debug > 0:
                    print("Action", action,  "failed")
                break
            if done > threshold:                        # behavior completed
                if self.debug > 0:
                    print("Action", action,  "completed")
                return True                             # action completed
            if maxsteps > 0 and nsteps > maxsteps:      # timeout
                if self.debug > 0:
                    print("Max number of steps reached: exiting", nsteps, ">", maxsteps)
                break
        return False

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

