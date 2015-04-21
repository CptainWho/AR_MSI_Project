""" Module
Module Description:
"""

__project__ = 'Aufgabenblatt 1'
__module__  = 'A2'
__author__  = 'Philipp Lohrer'
__email__   = 'plohrer@htwg-konstanz.de'
__date__    = '21.04.2015'

__version__ = '0.1'

# Imports
#################################################################
# Standard library imports
import numpy as np
from math import *
# Local imports
from HTWG_Robot_Simulator_V1 import (emptyWorld, Robot)
#################################################################

def straightDrive(v, l):
    [x, y, theta] = myRobot.getTrueRobotPose()
    dist = 0
    while l - dist > 0:
        myRobot.move([v, 0])
        x_old = x
        y_old = y
        [x, y, theta] = myRobot.getTrueRobotPose()
        dist += sqrt((x-x_old)**2 + (y-y_old)**2)


def curveDrive(v, r, deltatheta):
    omega = v / float(r)
    if deltatheta < 0:
        omega *= -1
    [x, y, theta] = myRobot.getTrueRobotPose()
    theta_ges = 0
    while abs(deltatheta) - abs(theta_ges) > 0:
        theta_old = theta
        myRobot.move([v, omega])
        [x, y, theta] = myRobot.getTrueRobotPose()
        if deltatheta < 0:
            theta_ges += (theta_old - theta) % (2 * pi)
        else:
            theta_ges += (theta - theta_old) % (2 * pi)


# Create empty World and new Robot
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 10
set_robot_opt['y'] = 10
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# Robot parameters
myRobot._k_d = 0
myRobot._k_drift = 0
myRobot._k_theta = 0

curveDrive(0.5, 4, -2*pi)
curveDrive(0.5, 2, 2*pi)
straightDrive(0.5, 5)
curveDrive(0.5, 2, pi)
straightDrive(0.5, 5)
curveDrive(0.5, 2, pi)

# Simulation schliessen:
myWorld.close()
