# -*- coding: utf-8 -*-
""" Module Description:

"""

__project__ = 'Exercise 5'
__module__ = 'ObstacleAvoidance'
__author__ = 'Philipp Lohrer'
__email__ = 'plohrer@htwg-konstanz.de'
__date__ = '15.07.2015'

# Changelog:

__version__ = '1.0'

# Imports
#################################################################
# Standard library imports
import numpy as np
# Local imports
from Exercise5.obstacle_avoidance import HistogramGrid as HG
#################################################################

class ObstacleAvoidance:
    """ Class description:

    """

    def __init__(self, robot, robot_loc, plot_grid=False):
        """
        :param robot:
        :param robot_loc:
        :param plot_grid:
        :return:
        """

        self.robot = robot
        self.robot_loc = robot_loc
        self.histogram_grid = HG.HistogramGrid(5, 5, cell_size=0.1, hist_threshold=10.0, plot_grid=plot_grid)
        self.plot_grid = plot_grid

        # Set start_position for robot
        robot_point = robot_loc.get_robot_point(est=False)
        self.robot_pos_x_old = robot_point[0]
        self.robot_pos_y_old = robot_point[1]
        self.x_residual = 0
        self.y_residual = 0

    def avoid_obstacle(self, target_point):
        """
        :param target_point:    [x, y]
        :return:                [v, omega] or None if no obstacle was found
        """

        # Get sensor angles, starting from -pi/2
        sensor_angles = np.asarray(self.robot.getSensorDirections())
        # Get sensor distances
        sensor_distances = np.asarray(self.robot.sense())

        # 1. Shift HistogramGrid according to relative movements of robot
        # 1.1. Get robot position and orientation
        robot_pos_x, robot_pos_y, robot_theta = self.robot_loc.get_robot_position()
        # 1.2. Calculate dx & dy
        dx = robot_pos_x - self.robot_pos_x_old - self.x_residual
        dy = robot_pos_y - self.robot_pos_y_old - self.y_residual
        # 1.3. Shift HistogramGrid
        self.x_residual, self.y_residual = self.histogram_grid.move_grid(dx, dy, debug=False)
        # 1.4 Set new position as old position
        self.robot_pos_x_old = robot_pos_x
        self.robot_pos_y_old = robot_pos_y

        # 2. Add detected distances to HistogramGrid
        # 2.1. Convert sensor angles to angles in the world system
        sensor_angles += robot_theta
        # 2.2. Add all detected distances to the HistogramGrid
        value_set = []
        for j, dist in enumerate(sensor_distances):
            if dist is not None:
                # Add value to HistogramGrid
                # print 'Set Histogram value at dist: %0.2f, angle: %0.2f' % (dist, sensor_angles[j])
                value_set.append(self.histogram_grid.set_value(dist, sensor_angles[j], debug=False))
            # else:
            #     # No obstacle was detected with the sensor-measurement -> reset corresponding cells
            #     self.histogram_grid.reset_value(sensor_angles[j])
        # 3. Perform path-finding with the resulting histogram
        # 3.1 If at least 1 sensor value was set in the histogram, run avoid_obstacle
        if np.any(value_set):
            v, omega = self.histogram_grid.avoid_obstacle(self.robot_loc, target_point, path='edge', debug=False)
        else:
            # No value was set in the histogram -> there's no obstacle in histogram range -> use old v & omega
            return None

        # 4. Optional: Visualize HistogramGrid and/or Histogram
        if self.plot_grid:
            self.histogram_grid.draw_grid()
            self.histogram_grid.draw_hist()

        return [v, omega]