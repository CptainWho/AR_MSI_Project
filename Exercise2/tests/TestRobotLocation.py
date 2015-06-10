# -*- coding: utf-8 -*-
__author__ = 'Ecki'

import numpy as np
from math import *
from HTWG_Robot_Simulator_V1 import Robot as Robot, emptyWorld as loadedWorld
from Exercise2.util import RobotLocation

# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 5
set_robot_opt['y'] = 10
set_robot_opt['theta'] = pi
myWorld.setRobot(**set_robot_opt)

RobotLoc = RobotLocation.RobotLocation(myRobot)

print RobotLoc.get_robot_angle()*180/pi
print RobotLoc.get_positive_robot_angle()*180/pi

# close world by clicking
myWorld.close()
