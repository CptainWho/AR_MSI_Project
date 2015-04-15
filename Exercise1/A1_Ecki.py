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

# def curveDrive(v, r, delta_theta):
#     omega_0 = v / float(r)
#     steps = int(t / 0.1)
#      for step in range(steps):
#          myRobot.move([v, omega_0])


# Anzahl Zeitschritte n mit jeweils der Laenge T = 0.1 sec definieren.
# T laesst sich ueber die Methode myRobot.setTimeStep(T) einstellen.
# T = 0.1 sec ist voreingestellt.
n = 150

myRobot._k_d = 0
myRobot._k_drift = 0
myRobot._k_theta = 0


#for i in range(40):
#    myRobot.move([0, pi/2])

#curveDrive(0.5, 2.5, 1)

straightDrive(0.5, 5)
curveDrive(0.5, 2.5, pi)
straightDrive(0.5, 5)
curveDrive(0.5, 2.5, pi)
curveDrive(0.5, 2.5, -2*pi)


# Simulation schliessen:
myWorld.close()

