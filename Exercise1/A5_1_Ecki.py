# -*- coding: utf-8 -*-
__author__ = 'Ecki'

"""
Aufgabe 5.1
Realisieren Sie einen Linenverfolger followPolyline(v, poly), der einen Polygonzug poly mit der
Geschwindigkeit v abfaehrt.
Implementieren Sie zwei Verfahren:
- der Roboter stoppt bei jedem Eckpunkt des Polygonzugs, richtet sich aus und faehrt erst dann
auf den naechsten Eckpunkt zu die Geschwindigkeit v wird moeglichst konstant gehalten.
Testen Sie Ihren Linienverfolger in typischen Szenarien.
"""

from math import *
from HTWG_Robot_Simulator_V1 import emptyWorld
from HTWG_Robot_Simulator_V1 import Robot
import copy as c
from Exercise2_Bachelor import MatrixTansformation as mt
import numpy as np

""" Init """

# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 1, 2, -pi/2)

# tell robo where he starts
[x0, y0, theta0] = myRobot.getTrueRobotPose()
myRobot.setOdoPose(x0, y0, theta0)

""" Functions """

# robot drives direclty towards point p with v
# robot stops within defined tolerance tol
def goto(v, p, tol):
    k_omega = 0.5
    [x, y, theta] = getRobotPos()
    while outOfTol([x, y], p, tol):
        # target direction
        theta_target = atan2(p[1] - y, p[0] - x)
        # calculate omega
        omega = k_omega * diff(theta, theta_target)
        # move robot
        myRobot.move([v, omega])
        # get new position
        [x, y, theta] = getRobotPos()


def getRobotPos():
    return myRobot.getTrueRobotPose()
    #return myRobot.getOdoPose()


# calculate the angle difference from theta to theta_target
# positive is defined as counterclockwise
def diff(theta, theta_target):
    return (theta_target - theta + pi) % (2 * pi) - pi


# check whether point in within tolerance of target point
def outOfTol(p, p_target, tol):
    sqrDist = (p[0] - p_target[0])**2 + (p[1] - p_target[1])**2
    sqrTol = tol**2
    if(sqrDist > sqrTol):
        return True
    else:
        return False


# check whether angle is within tolerance of target angle
def outOfAngleTol(angle, angle_target, tol):
    angle_diff = abs(diff(angle, angle_target))
    if angle_diff > tol:
        return True
    else:
        return False


#rotates robot
def rotateToTargetPoint(omega, point, angle_tol):
    k_omega = 0.5

    # target direction
    [x, y, theta] = getRobotPos()
    theta_target = atan2(point[1] - y, point[0] - x)

    while outOfAngleTol(theta, theta_target, angle_tol):
        # calculate omega
        omega = k_omega * diff(theta, theta_target)
        # move robot
        myRobot.move([0, omega])

        [x, y, theta] = getRobotPos()
        theta_target = atan2(point[1] - y, point[0] - x)

# follows a straight line between p1 and p2 with speed of v
def followLine(p1, p2, v):
    k_p = 0.4 #define p of closed loop
    k_d = 0.8 #define d of closed loop
    tol = 0.1 #tolrance
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


def followPolyline(v, poly):
    tol = 0.1 # tolerance
    omega = 0.8 # rotation speed
    angle_tol = 0.01 # tolerance of angle

    p_old = poly[0]

    # set all the points of polyline into for loop
    for p in poly:
        if p == poly[0]:
            # first go to start point
            goto(v, poly[0], tol)
        else:
            # rotate robot and follow the line to next point
            rotateToTargetPoint(omega, p, angle_tol)
            followLine(p_old, p, v)
            p_old = p


""" Main """

#define polyline
polyline = [[2, 10], [10, 10], [10, 6], [18, 6], [9, 18]]
myWorld.drawPolyline(polyline)

followPolyline(0.6, polyline)

[x, y, theta] = myRobot.getOdoPose()
print x, y, theta

# close world by clicking
myWorld.close()