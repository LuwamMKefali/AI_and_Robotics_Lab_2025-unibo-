"""
============== UniBo: AI and Robotics 2025 ==============
Base code: HTN planning domain for Pyhop planner
To be customized in an incremental way for the different labs

(c) 2024-2025 Alessandro Saffiotti
"""

import pyhop

class State(pyhop.State):
    def __init__(self):
        self.__name__ = "s1"
        self.pos = {}           # named position of robot ('me') or of an objet
        self.room = {}          # room of robot or of an objet
        self.connects = {}      # doors' connectivity: pair of rooms


###############################################################################
# OPERATORS
# First argument is current state, then come the operator's parameters
# Return a new state, of False if the operator is not applicable
###############################################################################

def GoTo (state, target):
    state.pos['me'] = target
    return state

def Cross (state, door):
    if (state.pos['me'] != door):
        return False
    if (state.room['me'] == state.connects[door][0]):
        state.room['me'] = state.connects[door][1]
        return state
    if (state.room['me'] == state.connects[door][1]):
        state.room['me'] = state.connects[door][0]
        return state
    return False

pyhop.declare_operators(GoTo, Cross)


###############################################################################
# METHODS
# First argument is current state, then come the method's parameters
# Return a decomposition into a list of sub-tasks, or False if not applicable
# Primitive sub-tasks will match operators; others will match other methods
###############################################################################

# Method to navigate when we are already at the target

def move_in_place (state, target):
    if state.pos['me'] == target:
        return []
    else:
        return False

# Method to navigate when the target is in the same room

def move_in_room(state, target):
    if state.room['me'] == state.room[target]:
        return [('GoTo', target)]
    else:
        return False

# Helper function to find connecting doors

def doors_between (state, room1, room2):
    doors = []
    for d in state.connects:
        if (state.connects[d][0] == room1 and state.connects[d][1] == room2) or (state.connects[d][0] == room2 and state.connects[d][1] == room1) :
            doors.append(d)
    return doors

# Method to navigate when the target is in an adjacent room

def move_across_rooms (state, target):
    room1 = state.room['me']
    room2 = state.room[target]
    if room1 == room2:
        return False
    doors = doors_between(state, room1, room2)
    if doors == []:
        return False
    door = doors[0]
    return [('GoTo', door), ('Cross', door), ('GoTo', target)]
    

pyhop.declare_methods('navigate_to',  move_in_place, move_in_room, move_across_rooms)

