"""
============== UniBo: AI and Robotics 2025 ==============
Base code: internal map of the environment
To be customized in an incremental way for the different labs

(c) 2024-2025 Alessandro Saffiotti
"""
from math import sqrt


class WorldMap:
    """
    This is the static map of the environment where our robot lives
    It contains the given position of all named objects and their topological relations
    """
    geometry  = {}              # extent of rooms, as (bottomleftx, bottomlefty, xsize, yzise)
    topology  = {}              # relations between rooms and objects
    locations = {}              # poses and size of known objects, as (mt, mt, rad, radius)
    properties = {}             # properties of objects
    startpose = (0.0, 0.0, 0.0) # starting pose of robot in map's coordinates

    def find_room (self, pos):
        """
        Given an (x,y) position, find the room where it belongs
        For each room, it checks if (x,y) falls inside that room's bounding box
        """
        for room, bbox in self.geometry.items():
            if pos[0] < bbox[0] or pos[0] > bbox[0]+bbox[2]:
                continue
            if pos[1] < bbox[1] or pos[1] > bbox[1]+bbox[3]:
                continue
            return room
        return None

    def find_location (self, pos):
        """
        Given an (x,y) position, find its symbolic location
        For each object, if checks if (x,y) is close to it considering its radius
        If the given pos is not close to any object, return 'openspace'
        """
        for object, loc in self.locations.items():
            dx = pos[0] - loc[0]
            dy = pos[1] - loc[1]
            dist = sqrt(dx*dx + dy*dy)
            if dist < loc[3] + 1.0:
                return object
        return 'openspace'
    
    def find_doors (self):
        """
        Scan the topology to find what doors connect what rooms
        Return a list of triples (door, room1, room2)
        """
        doors = []
        for room1 in self.topology:
            for room2 in self.topology:
                if room1 == room2:
                    continue
                for object1 in self.topology[room1]:
                    for object2 in self.topology[room2]:
                        if object1 == object2:
                            doors.append((object1, room1, room2))
        return doors


""" 
Here is the specific map for our environment
"""

map = WorldMap()
map.geometry = {
    'room1' : [-11.0, -1.5, 12.0, 8.0],
    'room2' : [-11.0, -9.5, 8.0, 8.0],
    'room3' : [-3.0, -9.5, 4.0, 8.0],
    'room4' : [1.0, -9.5, 10.0, 16.0]
}
map.topology = {
    'room1' : ['bed1', 'wardrobe1','bookshelf1', 'bookshelf2', 'door2', 'door3'],
    'room2' : ['fridge1', 'sink1', 'stove1', 'door4', 'table4'],
    'room3' : ['entrance', 'door1', 'door3', 'door4'],
    'room4' : ['table1', 'table2', 'table3', 'door1', 'door2']
}

map.taxonomy = {
    'Room'      : ['room1', 'room2', 'room3', 'room4'],
    'Door'      : ['entrance', 'door1', 'door2', 'door3', 'door4'],
    'Chair'     : [],
    'Table'     : ['table1', 'table2', 'table3'],
    'Sofa'      : [],
    'Bed'       : ['bed1'],
    'Wardrobe'  : ['wardrobe1'],
    'Fridge'    : ['fridge1'],
    'Sink'      : ['sink1'],
    'Stove'     : ['stove1'],
    'Shelf'     : ['bookshelf1', 'bookshelf2'],
    'Closet'    : []
}

map.locations = {
    'table1'    : (5.0, 0.0, 0.0, 0.7),
    'table2'    : (5.0, 5.5, 0.0, 1.0),
    'table3'    : (5.0, -5.0, 0.0, 1.3),
    'table4'    : (-6.6, -4.8, 0.0, 0.8),
    'bed1'      : (-9.5, 1.5, 1.57, 1.6),
    'fridge1'   : (-7.0, -9.0, 3.14, 0.6),
    'sink1'     : (-10.4, -5.0, 1.57, 1.2),
    'stove1'    : (-7.0, -2.3, 0.0, 0.6),
    'wardrobe1' : (-8.0, 5.76, 0.0, 2.2),
    'entrance'  : (-1.0, -8.0, 0.0, 0.0),
    'bookshelf1': (-2.51, 6.4, 0.0, 0.4),
    'bookshelf2': (-3.6, 6.4, 0.0, 0.4),
    'door1'        : (1.0, -5.4, 0.0, 0.6),   
    'door2'        : (1.0, 2.6, 0.0, 0.6),   
    'door3'        : (-1.2, -1.2, 1.57, 0.6),   
    'door4'        : (-3.0, -5.4, 0.0, 0.6)
}
map.properties = {
    'door1' : 'closed',
    'door2' : 'closed',
    'door3' : 'closed',
    'door4' : 'closed'
}
