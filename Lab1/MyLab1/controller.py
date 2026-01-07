"""
============== UniBo: AI and Robotics 2025 ==============
Controller: Task 3 (position-based navigation, linear control)
Base code: dummy controller, performs the action decision parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024-2025 Alessandro Saffiotti
"""

from math import pi

class Controller ():
    def __init__(self, robot_pars):
        self.vlin = 0.0                  # current value for vlin control variable
        self.vrot = 0.0                  # current value for vrot control variable

        # control cycle time (same as toplevel default, 0.1s)
        # if you changed tcycle in toplevel, change this too
        self.tcycle = 0.1

        # square parameters
        self.vlin_cmd = 0.25           # m/s
        self.vrot_cmd = 0.6            # rad/s

        # duration for one side and one 90-degree turn
        # side length ~ vlin_cmd * side_time
        self.side_time = 2.0           # seconds (4.0s at 0.25 m/s ≈ 1.0 m)
        self.turn_time = (pi/2.0) / self.vrot_cmd   # seconds to rotate ~90 degrees

        # convert to number of control steps
        self.side_steps = int(self.side_time / self.tcycle)
        self.turn_steps = int(self.turn_time / self.tcycle)

        # finite state machine
        self.phase = "SIDE"            # "SIDE" or "TURN" or "DONE"
        self.phase_step = 0
        self.sides_done = 0

    def run (self, state, debug):
        """
        Square path generator:
        - 4 straight segments
        - 4 in-place left turns (90 degrees)
        """
        if self.phase == "DONE":
            self.vlin = 0.0
            self.vrot = 0.0
            return

        if self.phase == "SIDE":
            self.vlin = self.vlin_cmd
            self.vrot = 0.0
            self.phase_step += 1

            if self.phase_step >= self.side_steps:
                self.phase = "TURN"
                self.phase_step = 0

                if debug > 0:
                    print("Completed side:", self.sides_done + 1)

            return

        if self.phase == "TURN":
            self.vlin = 0.0
            self.vrot = self.vrot_cmd
            self.phase_step += 1

            if self.phase_step >= self.turn_steps:
                self.sides_done += 1
                self.phase_step = 0

                if self.sides_done >= 4:
                    self.phase = "DONE"
                    self.vlin = 0.0
                    self.vrot = 0.0
                    if debug > 0:
                        print("Square path completed")
                    return

                self.phase = "SIDE"
                if debug > 0:
                    print("Completed turn, starting next side")

            return

    def get_vlin (self):
        return self.vlin
    
    def get_vrot (self):
        return self.vrot
