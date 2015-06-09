from World import *

target_x = 2
target_y = 1

def buildWorld():
    world = World(10, 10)

    # top 1
    world.addLine(5, 3, 9, 3)

    # bottom 1
    world.addLine(5, 7, 9, 7)

    # side left top
    world.addLine(5, 3, 5, 4)
    # side left bottom
    world.addLine(5, 6, 5, 7)

    # side right top
    world.addLine(9, 3, 9, 7)

    world.addBox(target_x, target_y)

    #polyline = [[1,6],[10.5,6],[10.5,3]]
    #world.drawPolyline(polyline)

    return world


def get_target():
    return [target_x, target_y]