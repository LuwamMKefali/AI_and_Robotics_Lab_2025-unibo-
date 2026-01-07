#!/usr/bin/env python3
"""
============== UniBo: AI and Robotics 2025 ==============
Base code: start code for Lab 3 (action planning)
(c) 2024-2025 Alessandro Saffiotti
"""
import toplevel, pyhop, htn_domain

# Create an initial state

s1 = htn_domain.State()

s1.room['bed1'] = 'Room1'
s1.room['wardrobe'] = 'Room1'
s1.room['fridge'] = 'Room2'
s1.room['stove'] = 'Room2'
s1.room['sink'] = 'Room2'
s1.room['table1'] = 'Room2'
s1.room['table2'] = 'Room4'
s1.room['table3'] = 'Room4'

s1.connects['D1'] = ('Room3', 'Room4')
s1.connects['D2'] = ('Room1', 'Room4')
s1.connects['D3'] = ('Room1', 'Room3')
s1.connects['D4'] = ('Room2', 'Room3')

s1.pos['me'] = 'table2'
s1.room['me'] = 'Room4'


if __name__ == '__main__':
    toplevel = toplevel.TopLevelLoop(mypose = (2.0, -2.0, 0.0), debug = 1)

    print("Initial state:")
    pyhop.print_state(s1)

    myplan = pyhop.pyhop(s1, [('navigate_to', 'table3')], verbose=2)
#   myplan = pyhop.pyhop(s1, [('navigate_to', 'bed1')], verbose=2)
#   myplan = pyhop.pyhop(s1, [('navigate_to', 'stove')], verbose=2)

    print('')
    print("Plan:", myplan)

#   toplevel.start(behavior = 'GoTo', behavior_params = (6.0, 4.0, 0.0, 0.0))
