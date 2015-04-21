""" Module
Module Description:
"""

__project__ = 'Aufgabenblatt 1'
__module__  = 'A1'
__author__  = 'Philipp Lohrer'
__email__   = 'plohrer@htwg-konstanz.de'
__date__    = '15.04.2015'

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
    t = l / float(v)
    # Get Timestep T
    T = myRobot.getTimeStep()
    # Steps necessary to drive length l
    steps = int(t / T)
    for step in range(steps):
        myRobot.move([v, 0])


def curveDrive(v, r, deltatheta):
    b = abs(deltatheta) * r
    t = b / float(v)
    omega = deltatheta * v / float(b)
    # Get Timestep T
    T = myRobot.getTimeStep()
    # Steps necessary to drive curve
    steps = int(t / float(T))
    for step in range(steps):
        myRobot.move([v, omega])

# Create empty World and new Robot
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 1
set_robot_opt['y'] = 10
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# Robot parameters
myRobot._k_d = 0
myRobot._k_drift = 0
myRobot._k_theta = 0

# Rechteckfahrt
# straightDrive(0.5, 2.0)
# curveDrive(0.5, 2, pi / 2.0)
# straightDrive(0.5, 2.0)
# curveDrive(0.5, 2, pi / 2.0)
# straightDrive(0.5, 2.0)
# curveDrive(0.5, 2, pi / 2.0)
# straightDrive(0.5, 2.0)
# curveDrive(0.5, 2, pi / 2.0)

# Fahrspurwchsel
straightDrive(0.5, 5)
print('Kurve neg')
curveDrive(0.5, 2.5, -pi/2)
print('Kurve pos')
curveDrive(0.5, 2.5, pi/2)
straightDrive(0.5, 5)

# Close Simulation
myWorld.close()


