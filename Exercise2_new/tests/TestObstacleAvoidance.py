# -*- coding: utf-8 -*-
""" Module Description:
Tests for class HistogramGrid
"""

__project__ = 'Exercise 2'
__module__ = 'TestObstacleAvoidance'
__author__ = 'Philipp Lohrer'
__email__ = 'plohrer@htwg-konstanz.de'
__date__ = '05.06.2015'

__version__ = '0.1'

# Standard library imports
from math import pi
import numpy as np
# Local imports
from HTWG_Robot_Simulator_V1 import Robot as Robot, obstacleWorld_test as loadedWorld
from Exercise2_new.obstacle_avoidance import HistogramGrid
from Exercise2_new.util import Calculations as Calc


# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 12
set_robot_opt['y'] = 5
set_robot_opt['theta'] = pi / 4.0
myWorld.setRobot(**set_robot_opt)

# Create HistogramGrid
myHistogramGrid = HistogramGrid.HistogramGrid(5, 5, cell_size=0.1, hist_threshold=0.1)

# Set start_position for robot
robo_pos_x_old = 12
robo_pos_y_old = 5
x_residual = 0
y_residual = 0

# Target point
target = [5, 5]

counter = 0


for i in xrange(100):
    # Get sensor angles, starting from -pi/2
    sensor_angles = np.asarray(myRobot.getSensorDirections())
    # Get sensor distances
    sensor_distances = np.asarray(myRobot.sense())

    # Check if an obstacle was detected
    # If-clause is executed if at least 1 distance is not None
    if np.any(sensor_distances):
        obstacle_detected = True

        # 1. Shift HistogramGrid according to relative movements of robot
        # 1.1. Get robot position and orientation
        robo_pos_x, robo_pos_y, robo_theta = myRobot.getTrueRobotPose()
        print 'robo_pos_x = %0.2f, robo_pos_y = %0.2f' % (robo_pos_x, robo_pos_y)
        # 1.2. Calculate dx & dy
        dx = robo_pos_x - robo_pos_x_old - x_residual
        dy = robo_pos_y - robo_pos_y_old - y_residual
        # 1.3. Shift HistogramGrid
        x_residual, y_residual = myHistogramGrid.move_grid(dx, dy, debug=False)
        # 1.4 Set new position as old position
        robo_pos_x_old = robo_pos_x
        robo_pos_y_old = robo_pos_y

        # 2. Add detected distances to HistogramGrid
        # 2.1. Convert sensor angles to angles in the world system
        sensor_angles += robo_theta
        # 2.2. Add all detected distances to the HistogramGrid
        for j, dist in enumerate(sensor_distances):
            if dist is not None:
                # Add value to HistogramGrid
                myHistogramGrid.set_value(dist, sensor_angles[j])


        # 3. Perform path-finding with the resulting histogram
    # TODO

        # 4. Optional: Visualize HistogramGrid and/or Histogram
        # myHistogramGrid.draw_grid()
        # myHistogramGrid.draw_hist()

    else:
        obstacle_detected = False

    # Move robot
    myRobot.move(myHistogramGrid.avoid_obstacle(myRobot.getTrueRobotPose(), target, debug=True))


