from World import *

def buildWorld():
    world = World(10, 10)

    world.addLine(3,3,3,7)

    world.addBox(7,7)


    #polyline = [[1,6],[10.5,6],[10.5,3]]
    #world.drawPolyline(polyline)

    return world