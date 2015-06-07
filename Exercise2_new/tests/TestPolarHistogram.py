# -*- coding: utf-8 -*-
__author__ = 'Ecki'

import numpy as np
from math import *
from HTWG_Robot_Simulator_V1 import Robot as Robot, obstacleWorld2 as loadedWorld
from Exercise2_new.util import RobotLocation
from Exercise2_new.obstacle_avoidance import PolarHistogram

# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 7
set_robot_opt['y'] = 5
set_robot_opt['theta'] = 0.1
myWorld.setRobot(**set_robot_opt)

robot_loc = RobotLocation.RobotLocation(myRobot)
polar_hist = PolarHistogram.PolarHistogram(myRobot, robot_loc)

histogram = polar_hist.generateHistFromSensors()
minima = polar_hist.locate_minima(histogram)

print polar_hist.generateHistFromSensors()
print minima
print minima[0][0]*180/pi
print minima[0][1]*180/pi
print minima[0][2]*180/pi

# close world by clicking
myWorld.close()