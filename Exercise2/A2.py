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
from HTWG_Robot_Simulator_V1 import Robot, emptyWorld as loadedWorld
from StateMachine import StateMachine
import BasicMovement as BasicMovement
import Braitenberg as Braitenberg
import RobotNavigation as RobotNavigation

#################################################################

# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 3
set_robot_opt['y'] = 7
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)


# Set StateMachine
robot_navigation = RobotNavigation.RobotNavigation(myRobot)
state_machine = StateMachine(myRobot, robot_navigation)
basic_mov = BasicMovement.BasicMovement(myRobot)
braitenberg = Braitenberg.Braitenberg(myRobot)

#define polyline
polyline = [[4, 8], [10, 8], [10, 6], [13, 6], [9, 13], [9, 14], [9, 8]]
myWorld.drawPolyline(polyline)

robot_navigation.setPolyline(polyline)

target_reached = False
states = {'NoObstacle', 'Obstacle', 'CornerReached', 'TargetReached'}
[v, omega] = [0, 0]

while not target_reached:
    state = state_machine.next_state()
    if state in states:
        if state == 'NoObstacle':
            [v, omega] = basic_mov.followLine(robot_navigation.getLastPoint(), robot_navigation.getNextPoint(), 0.6)

        if state == 'Obstacle':
            [v, omega] = braitenberg.beScary()

        if state == 'CornerReached':
            [v, omega] = basic_mov.rotateToTargetPoint(robot_navigation.getNextPoint(), 0.001)

        if state == 'TargetReached':
            target_reached = True
            [v, omega] = [0, 0]

        myRobot.move([v, omega])

# close world by clicking
myWorld.close()
