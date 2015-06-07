from World import *

def buildWorld():
    world = World(20, 20)

    # top 1 left
    world.addLine(10, 7, 11, 7)
    # top 2 right
    world.addLine(13, 7, 14, 7)

    # bottom 1 left
    world.addLine(10, 3, 11, 3)
    # bottom 2 right
    world.addLine(13, 3, 14, 3)

    # side left top
    world.addLine(10, 3, 10, 4)
    # side left bottom
    world.addLine(10, 7, 10, 6)

    # side right top
    world.addLine(14, 3, 14, 4)
    # side right bottom
    world.addLine(14, 7, 14, 6)

    #polyline = [[1,6],[10.5,6],[10.5,3]]
    #world.drawPolyline(polyline)

    return world