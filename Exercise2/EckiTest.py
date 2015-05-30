__author__ = 'Ecki'

# Imports
#################################################################
# Standard library imports
import numpy as np
from math import *
# Local imports
from HTWG_Robot_Simulator_V1 import RobotFrontLasers as Robot, obstacleWorld1 as loadedWorld
from StateMachine import StateMachine
import BasicMovement as BasicMovement
import Braitenberg
import RobotNavigation
import PolarHistogram

#################################################################

# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 10
set_robot_opt['y'] = 10
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)


# Set StateMachine
RobotNav = RobotNavigation.RobotNavigation(myRobot)
state_machine = StateMachine(myRobot, RobotNav)
basic_mov = BasicMovement.BasicMovement(myRobot)
braitenberg = Braitenberg.Braitenberg(myRobot)
PolarHist = PolarHistogram.PolarHistogram(myRobot, RobotNav)


p = [12, 8]
myWorld.drawCircle((p[0], p[1]))
print RobotNav.getAngleFromRobotToPoint(p) *180/pi

start = -10
stop = 10
angle = -11

print RobotNav.AngleInRange(start*pi/180, stop*pi/180, angle*pi/180, False, 360*pi/180)


# close world by clicking
myWorld.close()