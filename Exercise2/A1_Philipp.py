""" Module
Module Description:
Braitenberg vehicle
"""

__project__ = 'Aufgabenblatt 2'
__module__  = 'A1'
__author__  = 'Philipp Lohrer'
__email__   = 'plohrer@htwg-konstanz.de'
__date__    = '03.05.2015'

__version__ = '1.0'

# Imports
#################################################################
# Standard library imports
import numpy as np
from math import *
from datetime import (datetime, timedelta)
# Local imports
import Robot2
from HTWG_Robot_Simulator_V1 import obstacleWorld
#################################################################


# Create obstacleWorld and new Braitenberg Robot
myWorld = obstacleWorld.buildWorld()
myRobot = Robot2.Robot(braitenberg=True)
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 1
set_robot_opt['y'] = 10
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# Let robot move for x seconds
runtime = timedelta(seconds=5.0)
starttime = datetime.now()
while datetime.now() - starttime < runtime:
    myRobot.move([0, 0])


# # Set noise to zero
# myRobot._k_d = 0
# myRobot._k_drift = 0
# myRobot._k_theta = 0

# close world by clicking
myWorld.close()
