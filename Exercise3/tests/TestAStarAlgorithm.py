# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
from datetime import datetime
from HTWG_Robot_Simulator_V1 import Robot as Robot, officeWorld as loadedWorld
from Exercise3.navigation import AStarAlgo as AStarAlgo, Brushfire
from Exercise3.util import RobotLocation
from Exercise3.movements import CarrotDonkey


# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 5
set_robot_opt['y'] = 9
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# create instances
robot_loc = RobotLocation.RobotLocation(myRobot)
occupancy_grid = myWorld.getOccupancyGrid(0.125)
brushfire = Brushfire.Brushfire(occupancy_grid, myRobot)
brushfire.apply_brushfire(adjacency=4, safety_distance=0.5)
occupancy_grid.drawGrid()

a_star = AStarAlgo.AStarAlgorithm(occupancy_grid)

# define points
start_point = robot_loc.get_robot_point()
end_point = [10, 4]
myWorld.addBox(end_point[0], end_point[1])
# occupancy_grid.drawGrid()
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld)

t_start = datetime.now()
polyline = a_star.dijkstra_algorithm(start_point, end_point, adjacency=8)
t_delta = (datetime.now() - t_start).total_seconds()
print 'Found shortest path in: %0.5f seconds' % t_delta

print "Shortest path found! --> Draw polyline"
myWorld.drawPolyline(polyline)

carrot_donkey.setCarrotPosition(polyline[0])
for p_next in polyline:
    while carrot_donkey.carrot_pos != p_next:
        carrot_donkey.moveCarrotToPoint(p_next, 0.5)
        myRobot.move(carrot_donkey.followCarrot())

myWorld.close()