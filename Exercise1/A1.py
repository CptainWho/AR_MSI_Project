""" Module
Module Description:
"""

__project__ = 'Aufgabenblatt 1'
__module__  = 'A1'
__author__  = 'Philipp Lohrer'
__email__   = 'plohrer@htwg-konstanz.de'
__date__    = '015.04.2015'

__version__ = '0.1'

#################################################################
## Changelog:
#
#################################################################
## Imports
#################################################################
# Standard library imports
from math import *
# Local imports
import emptyWorld
import Robot
import World

#################################################################

# Create empty World and new Robot
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 2
set_robot_opt['y'] = 5.5
set_robot_opt['theta'] = pi/2
myWorld.setRobot(**set_robot_opt)

# Close Simulation
myWorld.close()

#test

