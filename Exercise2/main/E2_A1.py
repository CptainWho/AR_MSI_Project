# -*- coding: utf-8 -*-
""" Module Exercise2_old Aufgabe1
Module Description:
Realisieren Sie einen Roboter, der mit Hilfe der Abstandssensoren mit einem Braitenberg-
Verfahren Hindernissen ausweicht. Zudem soll der Roboter in hindernisfreien Bereichen mit einer
bestimmten Wahrscheinlichkeiten zufällige Drehungen durchführen.
"""

__project__ = 'Exercise 2'
__module__ = 'E2_A1'
__author1__ = 'Ecki'
__author2__ = 'Philipp Lohrer'
__date__ = '30.05.2015'

__version__ = '1.0'


# Standard library imports
from math import *
from datetime import datetime, timedelta
# Local imports
from HTWG_Robot_Simulator_V1 import obstacleWorld
from HTWG_Robot_Simulator_V1 import officeWorld
from HTWG_Robot_Simulator_V1 import Robot
from Exercise2.obstacle_avoidance import Braitenberg as Bb


# Create obstacleWorld and new Robot
myWorld = obstacleWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 5.5
set_robot_opt['y'] = 6
set_robot_opt['theta'] = pi/2.0
myWorld.setRobot(**set_robot_opt)

# Initialize Braitenberg behaviour
myBB = Bb.Braitenberg(myRobot)

# Let robot move for x seconds
runtime = timedelta(seconds=25.0)
start_time = datetime.now()
while datetime.now() - start_time < runtime:
    myRobot.move(myBB.be_scary())

# close world by clicking
myWorld.close()