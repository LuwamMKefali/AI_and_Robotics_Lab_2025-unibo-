#!/usr/bin/env python3
"""
============== UniBo: AI and Robotics 2025 ==============
Base code: start code for Lab 2 (fuzzy navigation behaviors)
(c) 2024-2025 Alessandro Saffiotti
"""
import toplevel

if __name__ == '__main__':
    toplevel = toplevel.TopLevelLoop(mypose = (2.0, -2.0, 0.0), debug = 2)
    toplevel.start(behavior = 'GoToTarget', behavior_params = (4.0, -1.0, 0.0, 0.0))
#   toplevel.start(behavior = 'Wander', behavior_params = None)
