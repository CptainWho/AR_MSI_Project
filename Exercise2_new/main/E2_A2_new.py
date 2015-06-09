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
__date__ = '09.06.2015'

__version__ = '1.0'


# Standard library imports
from math import *
import numpy as np
np.set_printoptions(threshold=np.nan)
# Local imports
from HTWG_Robot_Simulator_V1 import Robot as Robot, obstacleWorld1_small as loadedWorld
from Exercise2_new.util import StateMachine, Transitions
from Exercise2_new.movements import BasicMovement
from Exercise2_new.obstacle_avoidance import HistogramGrid
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
histogram_grid = HistogramGrid.HistogramGrid(5, 5, cell_size=0.1, hist_threshold=0.5)

target_reached = False
states = {'NoObstacle', 'Obstacle', 'CornerReached', 'TargetReached'}
[v, omega] = [0, 0]
# Set start_position for robot
robot_point = robot_loc.get_robot_point()
robot_pos_x_old = robot_point[0]
robot_pos_y_old = robot_point[1]
x_residual = 0
y_residual = 0

while not target_reached:
    next_point = polyline.get_next_point()
    state = state_machine.next_state()
    if state in states:
        if state == 'NoObstacle':
            [v, omega] = basic_mov.follow_line(polyline.get_last_point(), next_point, 0.5)

        if state == 'Obstacle':
            # Get sensor angles, starting from -pi/2
            sensor_angles = np.asarray(myRobot.getSensorDirections())
            # Get sensor distances
            sensor_distances = np.asarray(myRobot.sense())

            print 'DEBUG state Obstacle'
            print sensor_distances

            # 1. Shift HistogramGrid according to relative movements of robot
            # 1.1. Get robot position and orientation
            robot_pos_x, robot_pos_y, robot_theta = robot_loc.get_robot_position()
            # 1.2. Calculate dx & dy
            dx = robot_pos_x - robot_pos_x_old - x_residual
            dy = robot_pos_y - robot_pos_y_old - y_residual
            # 1.3. Shift HistogramGrid
            x_residual, y_residual = histogram_grid.move_grid(dx, dy, debug=False)
            # 1.4 Set new position as old position
            robot_pos_x_old = robot_pos_x
            robot_pos_y_old = robot_pos_y

            # 2. Add detected distances to HistogramGrid
            # 2.1. Convert sensor angles to angles in the world system
            sensor_angles += robot_theta
            # 2.2. Add all detected distances to the HistogramGrid
            for j, dist in enumerate(sensor_distances):
                if dist is not None:
                    # Add value to HistogramGrid
                    print 'Set Histogram value at dist: %0.2f, angle: %0.2f' % (dist, sensor_angles[j])
                    histogram_grid.set_value(dist, sensor_angles[j], debug=True)

            # 3. Perform path-finding with the resulting histogram
            v, omega = histogram_grid.avoid_obstacle(robot_loc.get_robot_position(), next_point, debug=False)

            # 4. Optional: Visualize HistogramGrid and/or Histogram
            # histogram_grid.draw_grid()
            # histogram_grid.draw_hist()

        if state == 'CornerReached':
            [v, omega] = basic_mov.rotate_to_target_point(polyline.get_next_point())

        if state == 'TargetReached':
            target_reached = True
            [v, omega] = [0, 0]


        myRobot.move([v, omega])


# close world by clicking
myWorld.close()
