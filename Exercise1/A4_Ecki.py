# -*- coding: utf-8 -*-
__author__ = 'Ecki'

"""
Aufgabe 4
Realisieren Sie eine Steuerung goto(v, p, tol), die den Roboter auf den Punkt p mit der
Geschwindigkeit v zusteuert, wobei der Punkt p lediglich mit einer gewissen Toleranz tol errreicht
werden muss.
"""

from math import *
from HTWG_Robot_Simulator_V1 import emptyWorld
from HTWG_Robot_Simulator_V1 import Robot
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
    #return myRobot.getOdoPose()
    return myRobot.getTrueRobotPose()


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

""" Main """

goto(0.4, [10, 10], 0.1)
[x, y, theta] = myRobot.getOdoPose()
print x, y, theta

# close world by clicking
myWorld.close()
