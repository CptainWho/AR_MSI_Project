# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
from HTWG_Robot_Simulator_V1 import Robot as Robot, officeWorld as loadedWorld
from Exercise3.robot_navigation import AStarAlgorithm
from Exercise3.util import RobotLocation
from Exercise3.movements import CarrotDonkey


# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 2
set_robot_opt['y'] = 2
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# create instances
occupancy_grid = myWorld.getOccupancyGrid(0.5)
robot_loc = RobotLocation.RobotLocation(myRobot)
a_star = AStarAlgorithm.AStarAlgorithm(occupancy_grid)

# define poits
start_point = a_star.match_in_grid(robot_loc.get_robot_point())
end_point = [10, 9]
myWorld.drawCircle((end_point[0], end_point[1]))
occupancy_grid.drawGrid()
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld)


a_star.shortest_path(start_point, end_point, myWorld, myRobot)
print a_star.closed_list.list


###############
#a_star.closed_list.set_list(list)

print "Linie zeichnen"

polyline = []
[point, p] = a_star.closed_list.list[-1]
while point != start_point:
    polyline.append(point)
    [point, p] = a_star.closed_list.get_linked_point(p)
polyline.append(start_point)

polyline = polyline[::-1]
myWorld.drawPolyline(polyline)
###############
# close world by clicking

carrot_donkey.setCarrotPosition(polyline[0])
for p_next in polyline:
    while carrot_donkey.carrot_pos != p_next:
        carrot_donkey.moveCarrotToPoint(p_next, 0.5)
        myRobot.move(carrot_donkey.followCarrot())


myWorld.close()