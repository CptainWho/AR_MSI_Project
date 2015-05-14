# -*- coding: utf-8 -*-

__author__ = 'Ecki'

"""
Aufgabe 1
Realisieren Sie die folgenden beiden kinematischen Grundfertigkeiten des Roboters:
- curveDrive(v, r, Δθ)
- straightDrive(v, l)
curveDrive bewegt den Roboter solange auf einem Kreisbogen mit Radius r und Geschwindigkeit
v, bis sich seine Orientierung um Δθ geändert hat. straightDrive bewegt den Roboter eine gerade
Strecke der Länge l mit der Geschwindigkeit v. Definieren Sie eine Folge von Geschwindigkeitsbefehlen,
die die gewünschte Bewegung umsetzt. Beachten Sie dabei, dass Geschwindigkeit und
Rotationsgeschwindigkeit beim Roboter begrenzt sind.
Gehen Sie zunächst davon aus, dass der Roboter die gewünschten Geschwindigkeiten korrekt
umsetzt (motionNoise auf 0 setzen) und realisieren Sie Kreis- und Rechteckfahrten und einen
Fahrspurwechsel.
Gehen Sie nun davon aus, dass die gewünschten Geschwindigkeiten nicht korrekt umgesetzt
werden (motionNoise werden auf die voreingestellten Werte gesetzt). Was beobachten Sie?
"""

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

