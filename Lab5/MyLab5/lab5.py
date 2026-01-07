#!/usr/bin/env python3
"""
============== UniBo: AI and Robotics 2025 ==============
Base code: start code for Lab 5 (semantic maps)
(c) 2024-2025 Alessandro Saffiotti
"""
import toplevel, knowledge_graph


if __name__ == '__main__':
    toplevel = toplevel.TopLevelLoop(debug = 1)

    kg = knowledge_graph.KnowledgeGraph()

    # example of deducing and printing the objects in room1
    kg.reason()
    kg.query_entity_types("room1", True)
    print(kg.objects_in_room("room1", True))

    # Example of changing the location of box2 to become wardrobe1
    result = kg.object_room("box2")
    old_room = result[0]
    old_room = str(old_room).split("/")[-1][:-4]
    kg.remove("box2", "hasLocation", old_room)
    kg.add("box2", "hasLocation", "wardrobe1")
    kg.reason()
    result2 = kg.object_room("box2", True)
    result2.extend(kg.object_room("box2", False))
    if(len(result2)) >0:
            new_location = list(result2)[0]
            new_location = str(new_location).split("/")[-1][:-4]
    print(new_location)

#   toplevel.start([('transport', 'box3', 'table3')])

