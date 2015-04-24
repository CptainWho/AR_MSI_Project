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
from Excercise2_Bachelor import MatrixTansformation as mt
import numpy as np
import copy as c

# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 1, 2, 0)

# tell robo where he starts
[x0, y0, theta0] = myRobot.getTrueRobotPose()
myRobot.setOdoPose(x0, y0, theta0)

""" Functions """

def getRobotPos():
    return myRobot.getTrueRobotPose()
    #return myRobot.getOdoPose()


def followLine(p1, p2):
    v = 0.5 #define robot speed
    k_p = 0.4 #define p of closed loop
    k_d = 0.8 #define d of closed loop
    tol = 0.5 #tolrance
    e_old = 0 #error start value

    # define coordinates transformation on line
    angle = atan2(p2[1] - p1[1], p2[0] - p1[0])
    rotMat = mt.rot(angle)
    t = [[p1[0]], [p1[1]]]
    T_OP = mt.transform(t, rotMat)
    T_PO = np.linalg.inv(T_OP)

    while outOfTol(getRobotPos(), p2, tol):

        # Robot position in coordinate system of the line
        [x, y, theta] = getRobotPos() # first get global position
        R_O = [x, y, 1] # make it homogeneous
        R_P = np.dot(T_PO, R_O)

        # distance is y in line coordinate system
        e = R_P[1]

        # closed-loop
        de_dt = (e - e_old) / myRobot.getTimeStep()
        omega = - k_p * e - k_d * de_dt

        e_old = c.copy(e)

        myRobot.move([v, omega])



# check whether point in within tolerance of target point
def outOfTol(p, p_target, tol):
    sqrDist = (p[0] - p_target[0])**2 + (p[1] - p_target[1])**2
    sqrTol = tol**2
    if(sqrDist > sqrTol):
        return True
    else:
        return False

""" Main """

# define line
p1 = [x0 + 1, y0 - 1]
p2 = [18, 18]

# draw the line
polyline = [[p1[0], p1[1]], [p2[0], p2[1]]]
myWorld.drawPolyline(polyline)

# follow the line
followLine(p1, p2)

# close world by clicking
myWorld.close()