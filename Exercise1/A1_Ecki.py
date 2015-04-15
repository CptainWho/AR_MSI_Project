__author__ = 'Ecki'

from math import *
from HTWG_Robot_Simulator_V1 import emptyWorld
from HTWG_Robot_Simulator_V1 import Robot


# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 10, 10, 0)


def straightDrive(v, l):
    t = l / float(v)
    steps = int(t / 0.1)
    for step in range(steps):
        myRobot.move([v, 0])


def curveDrive(v, r, deltatheta):
    b = abs(deltatheta) * r
    t = b / float(v)
    omega = deltatheta * v / float(b)
    steps = int(t / 0.1)
    for step in range(steps):
        myRobot.move([v, omega])


# Abweicheungen Null setzen
myRobot._k_d = 0
myRobot._k_drift = 0
myRobot._k_theta = 0

# Kreisfahrt
curveDrive(1, 2.5, -2*pi)

myWorld.setRobot(myRobot, 10, 10, 0)

# Rechteckfahrt
straightDrive(0.5, 5)
curveDrive(0.2, 2, pi/2)
straightDrive(0.5, 5)
curveDrive(0.2, 2, pi/2)
straightDrive(0.5, 5)
curveDrive(0.2, 2, pi/2)
straightDrive(0.5, 5)
curveDrive(0.2, 2, pi/2)

myWorld.setRobot(myRobot, 1, 10, 0)

# Fahrspurwchsel
straightDrive(0.5, 5)
curveDrive(0.5, 2.5, -pi/2)
curveDrive(0.5, 2.5, pi/2)
straightDrive(0.5, 5)

# Simulation schliessen:
myWorld.close()

