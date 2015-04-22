__author__ = 'Ecki'
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Aufgabe 3
Realisieren Sie einen Linienverfolger followLine(p1, p2), der eine Strecke von Punkt p1 nach Punkt
p2 moeglichst genau verfolgt. Realisieren Sie zunaechst einen P-Regler, der den Abstand zur Linie
auf 0 regelt. Was beobachten Sie? Verbessern Sie Ihre Regelung, indem Sie einen PD-Regler
einsetzen.
"""



""" To Do """""


from math import *
from HTWG_Robot_Simulator_V1 import emptyWorld
from HTWG_Robot_Simulator_V1 import Robot
from Excercise2_Bachelor import MatrixTansformation as mt
import numpy as np
import copy as c

# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 1, 10, 0)

# tell robo where he starts
[x0, y0, theta0] = myRobot.getTrueRobotPose()
myRobot.setOdoPose(x0, y0, theta0)

""" Functions """

def getRobotPos():
    return myRobot.getTrueRobotPose()
    #return myRobot.getOdoPose()


def followLine(p1, p2):

    # define coordinates transformation on line
    angle = atan2(p2[1] - p1[1], p2[0] - p1[0])
    rotMat = mt.rot(angle)
    t = [[p1[0]], [p1[1]]]
    T_OP = mt.transform(t, rotMat)
    T_PO = np.linalg.inv(T_OP)

    # Robot position in coordinate system of the line
    [x, y, theta] = getRobotPos() # first get global position
    R_O = [x, y, 1] # make it homogeneous
    R_P = np.dot(T_PO, R_O)

    # distance is y in line coordinate system
    e = R_P[1]

    # [x, y, theta] = myRobot.getOdoPose()
    # # calculate distance to line
    # a = np.asarray(p2) - np.asarray(p1)
    # r_g = np.asarray(p1)
    # r_q = np.asarray([x, y])
    # diff = r_g - r_q
    # e = np.linalg.norm(np.cross(a, diff)) / np.linalg.norm(a)

    # closed-loop
    k_p = 0.4
    k_d = 0.8
    de_dt = (e - followLine.e_old) / myRobot.getTimeStep()
    omega = - k_p * e - k_d * de_dt

    followLine.e_old = c.copy(e)

    return omega
followLine.e_old = 0

# check whether point in within tolerance of target point
def outOfTol(p, p_target, tol):
    sqrDist = (p[0] - p_target[0])**2 + (p[1] - p_target[1])**2
    sqrTol = tol**2
    if(sqrDist > sqrTol):
        return True
    else:
        return False

def straightDrive(v, l):
    t = l / float(v)
    steps = int(t / 0.1)
    for step in range(steps):
        myRobot.move([v, 0])

""" Main """

p1 = [x0, y0 - 1]
p2 = [x0 + 10, y0 - 1]



# draw the line
polyline = [[p1[0], p1[1]], [p2[0], p2[1]]]
myWorld.drawPolyline(polyline)

while outOfTol(getRobotPos(), p2, 0.5):
    myRobot.move([0.2, followLine(p1, p2)])

# close world by clicking
myWorld.close()