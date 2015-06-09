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

__project__ = 'Exercise 2'
__module__ = 'E2_A2'
__author1__ = 'Ecki'
__author2__ = 'Philipp Lohrer'
__date__ = '31.05.2015'

__version__ = '1.0'


# Standard library imports
from math import *
# Local imports
from HTWG_Robot_Simulator_V1 import Robot as Robot, obstacleWorld1 as loadedWorld
from Exercise2_new.util import StateMachine, Transitions
from Exercise2_new.movements import BasicMovement
from Exercise2_new.obstacle_avoidance import PolarHistogram
from Exercise2_new.util import Polyline
from Exercise2_new.util import RobotLocation


# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
#set_robot_opt['x'] = 2
#set_robot_opt['y'] = 6
set_robot_opt['x'] = 7
set_robot_opt['y'] = 1
set_robot_opt['theta'] = pi/2
myWorld.setRobot(**set_robot_opt)

# define polyline
polyline = [[6, 2], [5, 3], [5, 15], [10, 16], [13, 6], [9, 13], [9, 14], [9, 8], [1, 18]]
# polyline = [[3, 5.5], [22, 5.5]]
# polyline = [[3, 6], [9.5, 6], [11, 2]]
myWorld.drawPolyline(polyline)

polyline = Polyline.Polyline(polyline)

# Set up StateMachine
transitions = Transitions.Transitions(myRobot, polyline)
state_machine = StateMachine.StateMachine(transitions)
basic_mov = BasicMovement.BasicMovement(myRobot)
robot_loc = RobotLocation.RobotLocation(myRobot)
polar_hist = PolarHistogram.PolarHistogram(myRobot, robot_loc)



target_reached = False
states = {'NoObstacle', 'Obstacle', 'CornerReached', 'TargetReached'}
[v, omega] = [0, 0]

while not target_reached:
    next_point = polyline.get_next_point()
    state = state_machine.next_state()
    if state in states:
        if state == 'NoObstacle':
            [v, omega] = basic_mov.follow_line(polyline.get_last_point(), next_point, 0.5)

        if state == 'Obstacle':
            [v, omega] = polar_hist.avoid_obstacle(next_point)

        if state == 'CornerReached':
            [v, omega] = basic_mov.rotate_to_target_point(polyline.get_next_point())

        if state == 'TargetReached':
            target_reached = True
            [v, omega] = [0, 0]

        myRobot.move([v, omega])


# close world by clicking
myWorld.close()
