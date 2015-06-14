# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
from HTWG_Robot_Simulator_V1 import Robot as Robot, officeWorld as loadedWorld
from Exercise3.util import RobotLocation

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

myWorld.addLine(0.4,0.1,0.4,0.5)

# create instances
occupancy_grid = myWorld.getOccupancyGrid(0.1)
robot_loc = RobotLocation.RobotLocation(myRobot)



print occupancy_grid.getValue(0.4, 0.3)
occupancy_grid.setValue(0.24,0.24,1)
occupancy_grid.drawGrid()
occupancy_grid.getValue(x, y)


# close world by clicking
myWorld.close()