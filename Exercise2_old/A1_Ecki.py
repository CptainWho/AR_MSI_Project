# -*- coding: utf-8 -*-
__author__ = 'Ecki'

"""
Aufgabe 1
Realisieren Sie einen Roboter, der mit Hilfe der Abstandssensoren mit einem Braitenberg-
Verfahren Hindernissen ausweicht. Zudem soll der Roboter in hindernisfreien Bereichen mit einer
bestimmten Wahrscheinlichkeiten zufällige Drehungen durchführen.
Aufgabe
"""

from math import *
from HTWG_Robot_Simulator_V1 import obstacleWorld
from HTWG_Robot_Simulator_V1 import officeWorld
from HTWG_Robot_Simulator_V1 import Robot
import Braitenberg as Bb
from datetime import datetime, timedelta

# Roboter in einer Welt positionieren:
myWorld = obstacleWorld.buildWorld()
#myWorld = officeWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 4.5, 6, pi/2)


""" Main """

myBB = Bb.Braitenberg(myRobot)

# Let robot move for x seconds
runtime = timedelta(seconds=35.0)
starttime = datetime.now()
while datetime.now() - starttime < runtime:
    myRobot.move(myBB.beScary())

# close world by clicking
myWorld.close()