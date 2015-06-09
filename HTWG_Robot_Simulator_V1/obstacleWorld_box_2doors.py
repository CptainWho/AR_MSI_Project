from World import *

target_x = 3
target_y = 2

def buildWorld():
    world = World(10, 10)

    # top 1
    world.addLine(5, 3, 9, 3)

    # bottom 1
    world.addLine(5, 7, 9, 7)

    # side left bootom
    world.addLine(5, 3, 5, 3.5)
    # side left middle
    world.addLine(5, 3.75, 5, 5)  # (narrow door, robot does not)
    # world.addLine(5, 4, 5, 4.5)  # (narrow door, robot still fits)
    # side left top (wide door)
    world.addLine(5, 6.5, 5, 7)

    # side right top
    world.addLine(9, 3, 9, 7)

    world.addBox(target_x, target_y)

    #polyline = [[1,6],[10.5,6],[10.5,3]]
    #world.drawPolyline(polyline)

    return world


def get_target():
    return [target_x, target_y]