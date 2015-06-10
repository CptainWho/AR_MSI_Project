# -*- coding: utf-8 -*-
__author__ = 'Ecki'

"""
Aufgabe 3
Realisieren Sie einen Linienverfolger followLine(p1, p2), der eine Strecke von Punkt p1 nach Punkt
p2 moeglichst genau verfolgt. Realisieren Sie zunaechst einen P-Regler, der den Abstand zur Linie
auf 0 regelt. Was beobachten Sie? Verbessern Sie Ihre Regelung, indem Sie einen PD-Regler
einsetzen.
"""

from math import *
from HTWG_Robot_Simulator_V1 import emptyWorld
from HTWG_Robot_Simulator_V1 import Robot
from Exercise2_Bachelor import MatrixTansformation as mt
from Exercise2_old import BasicMovement as bm
import numpy as np
import copy as c

# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 1, 2, 0)

""" Main """
myBM = bm.BasicMovement(myRobot)

# define line
p1 = [2, 1]
p2 = [18, 18]

# draw the line
polyline = [[p1[0], p1[1]], [p2[0], p2[1]]]
myWorld.drawPolyline(polyline)

while 1:
    myRobot.move(myBM.followLine(p1, p2, 0.4))

# close world by clicking
myWorld.close()