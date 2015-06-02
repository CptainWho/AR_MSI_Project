# -*- coding: utf-8 -*-
__author__ = 'Ecki'

import numpy as np
from math import *
from HTWG_Robot_Simulator_V1 import Robot as Robot, emptyWorld as loadedWorld
from Exercise2_new.movements import  CarrotDonkey
from Exercise2_new.util import Transitions
from Exercise2_new.util import RobotLocation

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

CD = CarrotDonkey.CarrotDonkey(myRobot, myWorld)
Trans = Transitions.Transitions(myRobot)
RobotLoc = RobotLocation.RobotLocation(myRobot)

CD.setCarrotPosition([5, 5])
#while not RobotLoc.robot_inside_tolerance(CD.get_carrot_position(), 0.1):
while 1:
    myRobot.move(CD.followCarrot())

# close world by clicking
myWorld.close()
