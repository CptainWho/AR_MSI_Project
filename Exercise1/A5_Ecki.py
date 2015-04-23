__author__ = 'Ecki'
#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Aufgabe 5
Realisieren Sie einen Linenverfolger followPolyline(v, poly), der einen Polygonzug poly mit der
Geschwindigkeit v abfaehrt.
Implementieren Sie zwei Verfahren:
- der Roboter stoppt bei jedem Eckpunkt des Polygonzugs, richtet sich aus und faehrt erst dann
auf den naechsten Eckpunkt zu die Geschwindigkeit v wird moeglichst konstant gehalten. Sobald der Roboter einen
Eckpunkt mit einer gewissen Toleranz erreicht hat, fae hrt er bereits auf den naechsten
Eckpunkt zu.
Testen Sie Ihren Linienverfolger in typischen Szenarien.
"""

from math import *
from HTWG_Robot_Simulator_V1 import emptyWorld
from HTWG_Robot_Simulator_V1 import Robot
from A4_Ecki import goto
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

def followPolyline(v, poly):
    tol = 0.5 # tolerance

    t=1
    # first go to start point
    while outOfTol(getRobotPos(), poly[0], 0.5):
        t += 1

    for i in poly:
        print i

""" Main """

#define polyline
polyline = [[2, 10], [10, 10], [10, 6], [18, 6]]
myWorld.drawPolyline(polyline)

goto(0.4, [10, 10], 0.1)
[x, y, theta] = myRobot.getOdoPose()
print x, y, theta

# close world by clicking
myWorld.close()