# -*- coding: utf-8 -*-
__author__ = 'Ecki'

import numpy as np
from math import *
from HTWG_Robot_Simulator_V1 import Robot, obstacleWorld2 as loadedWorld
import RobotNavigation
import PolarHistogram

# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 5
set_robot_opt['y'] = 7
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

RobotNav = RobotNavigation.RobotNavigation(myRobot)
PolarHist = PolarHistogram.PolarHistogram(myRobot, RobotNav)

#define target
target = [20.0, 4.0]
myWorld.drawCircle((target[0], target[1]))

# while not RobotNav.indsideTol(target, 0.2):
#     myRobot.move(PolarHist.avoidObstacle(RobotNav.getAngleToPoint(target)))

histogram = PolarHist.generateHistFromSensors()
test_histogram = [[1, 1], [2, 0], [3, 1], [4, 0], [5, 0], [6, 1], [7, 1], [8, 1]]
minima = PolarHist.locateMinima(histogram)




closest_angle = RobotNav.searchClosestAngle(1, RobotNav.getColumnFromList(2, minima))

print histogram
print minima
print closest_angle



# close world by clicking
myWorld.close()
