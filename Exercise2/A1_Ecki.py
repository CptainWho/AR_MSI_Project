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
from HTWG_Robot_Simulator_V1 import Robot
import Braitenberg as BB

# Roboter in einer Welt positionieren:
myWorld = obstacleWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 5, 5, pi/2)


""" Main """

myBB = BB.Braitenberg(myRobot)

while 1:
    myRobot.move(myBB.beScary())

# close world by clicking
myWorld.close()