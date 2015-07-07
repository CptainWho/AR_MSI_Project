# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from HTWG_Robot_Simulator_V1 import Robot as Robot, obstacleWorld1 as loadedWorld
from Exercise5.util import RobotLocation
from Exercise5.util import Calculations as Calc


# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 11
set_robot_opt['y'] = 7
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# create instances
robot_loc = RobotLocation.RobotLocation(myRobot)

# define points
start_point = robot_loc.get_robot_point()
end_point = [10, 4]
myWorld.addBox(end_point[0], end_point[1])

# Find shortest path from start_point to end_point
polyline = [[1, 2], [5, 5], [6, 7]]

myWorld.drawPolyline(polyline)

print robot_loc.get_relative_obstacle_points()

myWorld.close()
