# -*- coding: utf-8 -*-
""" Module Aufgabe 2
Module Description:
Erweitern Sie nun Ihren Linienverfolger followPolyline() um eine Hindernisvermeidung. Ein
vorgegebener Polygonzug soll möglichst genau abgefahren werden, wobei Hindernissen
ausgewichen werden muss. Die folgenden Abbildungen zeigen für vorgegebene Polygonzüge
(grün) typische Trajektorien (rot).
Hinweis: Hilfreich könnte sein, den Linenverfolger als endlichen Automaten mit folgenden
Zuständen zu realisieren:
- der Roboter fährt auf den nächsten Eckpunkt zu und hat kein Hindernis in unmittelbarer
Nähe in Fahrtrichtung.
- der Roboter fährt auf den nächsten Eckpunkt zu, sieht aber ein Hindernis in Fahrtrichtung:
er wählt aufgrund der Sensordaten eine neue Richtung, die möglichst nahe an der
Zielrichtung liegt, aber frei befahrbar ist (siehe auch Histogrammverfahren).
- der Roboter hat einen Eckpunkt erreicht und richtet sich auf den nächsten Eckpunkt aus.
Die Geschwindigkeit sollte aus Sicherheitsgründen umgekehrt proportional zur Rotationsgeschwindigkeit gewählt werden.
"""

__project__ = 'Aufgabenblatt 2'
__module__  = 'A2'
__author__  = 'Philipp Lohrer'
__email__   = 'plohrer@htwg-konstanz.de'
__date__    = '14.05.2015'

__version__ = '0.1'

# Imports
#################################################################
# Standard library imports
import numpy as np
from math import *
# Local imports
from HTWG_Robot_Simulator_V1 import Robot, obstacleWorld as obstacleWorld
from StateMachine import StateMachine

#################################################################

# Create obstacleWorld and new Robot
myWorld = obstacleWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 1
set_robot_opt['y'] = 10
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# Set StateMachine
state_machine = StateMachine(myRobot)

target_reached = False
states = {'NoObstacle', 'Obstacle', 'CornerReached', 'TargetReached'}

while not target_reached:
    state = state_machine.next_state()
    if state in states:
        if state == 'NoObstacle':
            pass

        if state == 'Obstacle':
            pass

        if state == 'CornerReached':
            pass

        if state == 'TargetReached':
            pass


