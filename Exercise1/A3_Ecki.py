__author__ = 'Ecki'
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Aufgabe 3
Realisieren Sie einen Linienverfolger followLine(p1, p2), der eine Strecke von Punkt p1 nach Punkt
p2 möglichst genau verfolgt. Realisieren Sie zunächst einen P-Regler, der den Abstand zur Linie
auf 0 regelt. Was beobachten Sie? Verbessern Sie Ihre Regelung, indem Sie einen PD-Regler
einsetzen.
"""






""" To Do """""


from math import *
from HTWG_Robot_Simulator_V1 import emptyWorld
from HTWG_Robot_Simulator_V1 import Robot
from Excercise2_Bachelor import MatrixTansformation as mt
import numpy as np

# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 1, 10, 0)

[x0, y0, theta0] = myRobot.getTrueRobotPose()
# tell robo where he starts
myRobot.setOdoPose(x0, y0, theta0)

""" Functions """

def followLine(p1, p2):
    # draw the line
    polyline = [[p1[0], p1[1]], [p2[0], p2[1]]]
    myWorld.drawPolyline(polyline)

    # define coordinates transformation on line
    rotMat = mt.rot(atan2(p2[1] - p1[1], p2[0] - p1[0]))
    mt.transform(p1, rotMat)


    [x, y, theta] = myRobot.getOdoPose()
    # calculate distance to line
    a = np.asarray(p2) - np.asarray(p1)
    r_g = np.asarray(p1)
    r_q = np.asarray([x, y])
    diff = r_g - r_q
    e = np.linalg.norm(np.cross(a, diff)) / np.linalg.norm(a)

    # closed-loop
    k_p = 0.3
    omega = - k_p * e

    return omega


def straightDrive(v, l):
    t = l / float(v)
    steps = int(t / 0.1)
    for step in range(steps):
        myRobot.move([v, 0])

""" Main """

p1 = [x0, y0-2]
p2 = [x0+10, y0-2]
followLine(p1, p2)

straightDrive(0.5, 5)
print myRobot.getOdoPose()

# close world by clicking
myWorld.close()