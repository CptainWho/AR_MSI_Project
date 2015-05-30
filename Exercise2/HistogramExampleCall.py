# -*- coding: utf-8 -*-
""" Module HistogramExampleCall
Beispielsaufruf/-ablauf für Erstellung eines Histograms, Auswertung und Ausgabe
"""

__project__ = 'Aufgabenblatt 2'
__module__  = 'A2'
__author__  = 'Philipp Lohrer'
__email__   = 'plohrer@htwg-konstanz.de'
__date__    = '28.05.2015'

__version__ = '0.1'

from math import pi
import numpy as np
from HTWG_Robot_Simulator_V1 import Robot
from HistogramGrid import HistogramGrid


myHistogramGrid = HistogramGrid(5, 5)
myRobot = Robot.Robot()
robo_pos_x_old = 0
robo_pos_y_old = 0


# Get sensor angles, starting from -pi/2
sensor_angles = np.asarray(myRobot.getSensorDirections())
# Get sensor distances
sensor_distances = np.asarray(myRobot.sense())

# Check if an obstacle was detected
# If-clause is executed if at least 1 distance is not None
if np.any(sensor_distances):
    obstacle_detected = True

    # 1. Shift HistogramGrid according to relative movement of robot
    # 1.1. Get robot position and orientation
    robo_pos_x, robo_pos_y, robo_theta = myRobot.getTrueRobotPose()
    # 1.2. Calculate dx & dy
    dx = robo_pos_x - robo_pos_x_old
    dy = robo_pos_y - robo_pos_y_old
    # 1.3. Shift HistogramGrid
    myHistogramGrid.move_grid(dx, dy)

    # 2. Add detected distances to HistogramGrid
    # 2.1. Convert sensor angles to angles in the world system
    sensor_angles += robo_theta
    # 2.2. Add all detected distances to the HistogramGrid
    for i, dist in enumerate(sensor_distances):
        if dist is not None:
            # Add value to HistogramGrid
            myHistogramGrid.set_value(dist, sensor_angles[i])

    # 3. Perform path-finding with the resulting histogram
# TODO

    # 4. Optional: Visualize HistogramGrid and/or Histogram
# TODO

else:
    obstacle_detected = False


