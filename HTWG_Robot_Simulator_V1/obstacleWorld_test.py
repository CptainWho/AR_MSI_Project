from World import *

def buildWorld():
    world = World(20, 20)

    world.addLine(5, 10, 5, 6)
    world.addLine(8, 9, 8, 7)


    #polyline = [[1,6],[10.5,6],[10.5,3]]
    #world.drawPolyline(polyline)

    return world