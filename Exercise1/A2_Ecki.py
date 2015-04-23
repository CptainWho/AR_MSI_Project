# -*- coding: utf-8 -*-
__author__ = 'Ecki'


"""
Aufgabe 2
Die tatsächliche Position des Roboters kann mit getTrueRobotPose() abgefragt werden. Die
Funktionen aus Aufgabe 1 sollen nun mit Hilfe dieser Funktion umgesetzt werden.
"""

from math import *
from HTWG_Robot_Simulator_V1 import emptyWorld
from HTWG_Robot_Simulator_V1 import Robot

# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 10, 10, pi/2)

def straightDrive(v, l):
    [x, y, theta] = myRobot.getTrueRobotPose()
    dist = 0
    while l - dist > 0:
        myRobot.move([v, 0])
        x_old = x
        y_old = y
        [x, y, theta] = myRobot.getTrueRobotPose()
        dist += sqrt((x-x_old)**2 + (y-y_old)**2)


def curveDrive(v, r, deltatheta):
    omega = v / float(r)
    if deltatheta < 0:
        omega *= -1
    [x, y, theta] = myRobot.getTrueRobotPose()
    theta_ges = 0
    while abs(deltatheta) - abs(theta_ges) > 0:
        theta_old = theta
        myRobot.move([v, omega])
        [x, y, theta] = myRobot.getTrueRobotPose()
        if deltatheta < 0:
            theta_ges += (theta_old - theta) % (2 * pi)
        else:
            theta_ges += (theta - theta_old) % (2 * pi)


# Abweicheungen Null setzen
myRobot._k_d = 0
myRobot._k_drift = 0
myRobot._k_theta = 0

curveDrive(0.5, 4, -4*pi)

# Simulation schliessen:
myWorld.close()