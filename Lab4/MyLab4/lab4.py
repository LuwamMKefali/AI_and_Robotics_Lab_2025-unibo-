#!/usr/bin/env python3
"""
============== UniBo: AI and Robotics 2025 ==============
Base code: start code for Lab 4 (robot plan execution)
(c) 2024-2025 Alessandro Saffiotti
"""
import toplevel


if __name__ == '__main__':
    toplevel = toplevel.TopLevelLoop(mypose = (2.0, -2.0, 0.0), debug = 1)

#   toplevel.start([('navigate_to', 'table3')])
    toplevel.start([('navigate_to', 'bed1')])
#   toplevel.start([('navigate_to', 'stove1')])
#   toplevel.start([('transport', 'box3', 'table3')])

