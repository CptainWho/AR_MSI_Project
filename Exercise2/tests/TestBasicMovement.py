# -*- coding: utf-8 -*-
__author__ = 'Ecki'

import numpy as np
from math import *
from HTWG_Robot_Simulator_V1 import Robot as Robot, emptyWorld as loadedWorld
from Exercise2.movements import BasicMovement
from Exercise2.util import Transitions
from Exercise2.util import RobotLocation

# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 1
set_robot_opt['y'] = 1
set_robot_opt['theta'] = pi
myWorld.setRobot(**set_robot_opt)

BasicMov = BasicMovement.BasicMovement(myRobot)
Trans = Transitions.Transitions(myRobot)
RobotLoc = RobotLocation.RobotLocation(myRobot)

polyline = [[5, 4], [5, 15], [10, 16], [13, 6], [9, 13], [9, 14], [9, 8], [3, 16]]
myWorld.drawPolyline(polyline)


#while not RobotLoc.robot_inside_tolerance(CD.get_carrot_position(), 0.1):
while 1:
    myRobot.move(BasicMov.follow_line(polyline[0], polyline[1], 0.3))

# close world by clicking
myWorld.close()
