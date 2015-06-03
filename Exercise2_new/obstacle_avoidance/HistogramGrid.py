# -*- coding: utf-8 -*-
""" Module Description:
Histogram Grid for obstacle avoidance.
"""

__project__ = 'Exercise 2'
__module__ = 'HistogramGrid'
__author__ = 'Philipp Lohrer'
__email__ = 'plohrer@htwg-konstanz.de'
__date__ = '30.05.2015'

__version__ = '0.9'

# Imports
#################################################################
# Standard library imports
import numpy as np
from math import *
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
# Local imports
from Exercise2_new.util import Calculations as Calc
#################################################################


class HistogramGrid:
    """ Class description:
    Creates an empty histogram grid which can be filled with values for obstacle avoidance
    For this purpose the minimums of the polar histogram can be retrieved and a route to
    avoid an detected obstacle can be calculated.
    Furthermore the histogram and the histogram grid can be displayed via matplotlib.
    """

    def __init__(self, width, height, cell_size=0.1, hist_resolution=10):
        """ Initialize grid
        :param width: int
        :param height: int
        :param cell_size: default 0.1
        :param hist_resolution: angle for each histogram sector, default: 10°
        :return: -
        """

        self.x_size = int(width/cell_size)
        self.y_size = int(height/cell_size)
        # If x/y_size is even, add 1
        if self.x_size % 2 == 0:
            self.x_size += 1
        if self.y_size % 2 == 0:
            self.y_size += 1
        self.grid = np.zeros((self.x_size, self.y_size), dtype=np.int)
        self.width = width
        self.height = height
        self.cell_size = float(cell_size)

        self.histogram = None
        self.hist_resolution = hist_resolution

        # Pyplot: enable interactive (= non-blocking) mode
        plt.ion()
        # Pyplot: create 2 subplots for histogram grid and polar histogram
        fig, (self.ax1, self.ax2) = plt.subplots(nrows=2, ncols=1)
        # Create plt-obj of matshow for faster data updating
        self.plt_grid = None
        # Initialize histogram with empty data
        self.plt_hist = None

    def _init_grid(self, axis):
        """ Initialize HistogramGrid
        :param axis: axis for plotting
        :return: pyplot matshow obj
        """

        plt_grid = axis.matshow(self.grid)

        return plt_grid

    def _init_hist(self, axis):
        """ Initialize histogram
        :param axis: axis for plotting
        :return: pyplot bar obj
        """
        # get sector angles and occupancy
        sector_angles, sector_occupancy = self.histogram
        # Extract sector angles as strings
        text_sector_angles = [np.array_str(angle, precision=2) for angle in sector_angles[0]]
        # Width of the bars
        width = self.hist_resolution / 2.0
        # Set up bars
        plt_hist = axis.bar(sector_angles[0], sector_occupancy[0], width=width)
        # add text for labels, title and axis ticks
        axis.set_xlabel('sector angles')
        axis.set_ylabel('occupancy value')
        axis.set_title('Histogram')
        axis.set_xticks(sector_angles[0])
        axis.set_xticklabels(text_sector_angles,  rotation='vertical')

        return plt_hist

    def avoid_obstacle(self, robot_pos, target_point, debug=False):
        """ Calculates nearest way in regard of given target point around detected obstacles
        :param robot_pos: [x,y]
        :param target_point: [x,y]
        :return: speed, angular velocity ([v,omega])
        """

        # Get direction to target_point
        target_angle = Calc.get_angle_from_robot_to_point(robot_pos, target_point)
        # Create polar histogram
        histogram = self.create_histogram()
        # Split histogram in sector_angles and sector_occupancy
        sector_angles, sector_occupancy = histogram
        # Search for minimum occupancy values and return indexes
        min_indexes = np.where(sector_occupancy == sector_occupancy.min())
        # Get sector_angles with minimum occupancy and transform them to radian
        angles_min_occupancy = sector_angles[min_indexes] / 180.0 * pi
        # Search angle closest to target_angle
        closest_angle = Calc.search_closest_angle(target_angle, angles_min_occupancy)

        # Calculate omega for closest_angle
        omega = 1.0 * Calc.diff(robot_pos[2], (closest_angle / 180.0 * pi))
        # TODO scale v with occupancy value
        v = 0.5

        if debug:
            print 'DEBUG: avoid_obstacle()'
            print '\ttarget angle: %0.2f' % target_angle * 180.0 / pi
            print '\tclosest angle: %0.2f' % closest_angle * 180.0 / pi
        return [v, omega]

    def move_grid(self, dx, dy, debug=False):
        """ Shift grid values according to robot motion dx, dy
        :param dx: relative robot movements in x direction
        :param dy: relative robot movements in y direction
        :return: x_residual, y_residual
        """

        x_shift = int(round(dx / self.cell_size))
        y_shift = int(round(dy / self.cell_size))

        # Calculate remaining (=residual) dx, dy
        x_residual = (x_shift - dx / self.cell_size) * self.cell_size
        y_residual = (y_shift - dy / self.cell_size) * self.cell_size

        # Shift array values via padding
        # np.pad(array_like,((before_axis1,after_axis1),(before_axis2,after_axis2)), ...
        # ... mode='constant')[column_start : column_stop, row_start : row_stop]

        if x_shift > 0:
            self.grid = np.pad(self.grid, ((0, 0), (0, x_shift)), mode='constant')[:, x_shift:]
        elif x_shift < 0:
            x_shift = abs(x_shift)
            self.grid = np.pad(self.grid, ((0, 0), (x_shift, 0)), mode='constant')[:, :-x_shift]

        if y_shift > 0:
            self.grid = np.pad(self.grid, ((y_shift, 0), (0, 0)), mode='constant')[:-y_shift, :]
        elif y_shift < 0:
            y_shift = abs(y_shift)
            self.grid = np.pad(self.grid, ((0, y_shift), (0, 0)), mode='constant')[y_shift:, :]

        if debug:
            print 'DEBUG: move_grid()'
            print '\tdx = %0.2f, dy = %0.2f' % (dx, dy)
            print '\t-> x_shift = %i, y_shift = %i' % (x_shift, y_shift)
            print '\t-> x_residual = %0.2f, y_residual = %0.2f' % (x_residual, y_residual)
            # print self.grid

        return x_residual, y_residual

    def set_value(self, r, theta, value=1, debug=False):
        """ Set grid value at r, theta
        :param r: radius
        :param theta: angle
        :param value: default 1
        :return: -
        """

        # Convert polar coordinates to cartesian coordinates
        x, y = Calc.polar_2_cartesian(r, theta)

        # Coord Transformation: Place origin in middle of coord-system and reverse y-axis
        xi = int(x/self.cell_size + self.x_size / 2.0)
        yi = int(-y/self.cell_size + self.y_size / 2.0)

        # Check if coordinates exceed grid boundaries
        if xi < 0 or xi >= self.x_size:
            return
        if yi < 0 or yi >= self.y_size:
            return
        self.grid[yi, xi] += value

        # DEBUG
        if debug:
            print 'DEBUG: set_value()'
            print '\tGiven polar coordinates'
            print '\tr = %0.2f, theta = %0.2f' % (r, theta)
            print '\tResulting cartesian coordinates:'
            print '\tx = %0.2f, y = %0.2f' % (x, y)
            print '\tGrid coordinates:'
            print '\txi = %i, yi = %i' % (xi, yi)
            print '\tResulting grid:'
            print self.grid

    def create_histogram(self, resolution=10, debug=False):
        """ Creates histogram and returns the angles with corresponding occupancy values
        :param resolution: angle for each sector, default: 10°
        :param debug: enable/disable debug-printing
        :return: numpy array([[sector_angles],[sector_occupancy]])
        """

        # Constants
        weight_const_a = 1.0
        weight_const_b = 1.0 / 5.0  # 1 / max. sense value

        # Create arrays for sector_angles ([angles]) and sector_occupancy ([empty])
        sectors = int(360/float(resolution) + 1.0)  # 360° included for easier loop usage
        sector_angles = np.array([x * resolution for x in xrange(sectors)])
        sector_occupancy = np.zeros(sectors)

        # Get occupied cells from histogram grid and save their indexes ( array([[y1,y2,...,yn],[x1,x2,..,xn]]) )
        occupied_cell_indexes = np.asarray(np.nonzero(self.grid))

        # Get occupancy_values of occupied cells
        occupancy_values = self.grid[occupied_cell_indexes[0], occupied_cell_indexes[1]]

        # Reverse Coord Transformation: Shift origin back to left upper corner of coord-system and reverse y-axis
        occupied_cell_indexes[1] = (occupied_cell_indexes[1] - (self.x_size - 1) / 2.0) * self.cell_size
        occupied_cell_indexes[0] = (-occupied_cell_indexes[0] + (self.y_size - 1) / 2.0) * self.cell_size

        # Transpose cell_indexes to get (x,y) tuples ( array([[y1,x1],[y2,x2],...[yn,xn]]) )
        occupied_cell_indexes_t = np.transpose(occupied_cell_indexes)

        # Convert cell indexes to polar coordinates, calculate weight_m and add it to the corresponding sector
        # distances = array([d1,d2,...d_n])
        distances = np.sqrt(occupied_cell_indexes[0]**2 + occupied_cell_indexes[1]**2)
        # angles = array([alpha1,alpha2,...,alpha_n])
        angles = np.arctan2(occupied_cell_indexes[0], occupied_cell_indexes[1]) / pi * 180.0
        # If angle < 0 add 360° to get range 0°...360°
        angles[angles < 0] += 360.0
        # Calculate weight for each occupied cell
        weights = occupancy_values**2 * (weight_const_a - weight_const_b * distances)

        # Add weights to corresponding sector n according to angle alpha
        for i in xrange(len(sector_angles) - 1):
            # Find indexes of all weight-angles which fit into current sector_angle
            indexes = np.argwhere([sector_angles[i] <= angle < sector_angles[i+1] for angle in angles])
            # Add sum of all indexed weights to current sector
            sector_occupancy[i] = np.sum(weights[indexes])

        # Generate histogram, discard 360° sector and its occupancy value (last element)
        histogram = np.array([[sector_angles[:-1]], [sector_occupancy[:-1]]])

        # DEBUG
        if debug:
            print 'DEBUG: get_histogram()'
            print '\tsector_angles:\n\t', sector_angles
            print'\tsector_occupancy:\n\t', sector_occupancy
            print'\toccupied_cell_indexes:\n\t', occupied_cell_indexes
            print'\toccupancy_values:\n\t', occupancy_values
            print'\toccupied_cell_indexes transformed:\n\t', occupied_cell_indexes
            print '\tdistances:\n\t', distances
            print '\tangles:\n\t', angles
            print '\toccupancy values:\n\t', occupancy_values
            print '\tweights:\n\t', weights
            print '\tsector_occupancy updated:\n\t', sector_occupancy
            print '\thistogram transposed:\n\t', histogram.transpose()

        return histogram

    def get_value(self, x, y):
        """ Get grid value at the coordinate (x,y)
        :param x: coord x
        :param y: coord y
        :return: grid value at (x,y)
        """

        if x < 0 or x > self.width:
            return
        if y < 0 or y > self.width:
            return
        xi = int(x/self.cell_size)
        yi = int(y/self.cell_size)

        return self.grid[yi, xi]

    def draw_grid(self, debug=False):
        """ Draw current grid
        :return: -
        """

        if debug:
            print 'DEBUG draw_grid()'
            print self.grid

        if self.plt_grid is None:
            self.plt_grid = self._init_grid(self.ax1)
        else:
            self.plt_grid.set_data(self.grid)
            self.plt_grid.autoscale()
        plt.show()

    def draw_hist(self):
        """ Draw given histogram
        :return: -
        """
        # get sector angles and occupancy
        self.histogram = self.create_histogram(resolution=self.hist_resolution)
        sector_angles, sector_occupancy = self.histogram
        if self.plt_hist is None:
            self.plt_hist = self._init_hist(self.ax2)
        else:
            [bar.set_height(sector_occupancy[0][i]) for i, bar in enumerate(self.plt_hist)]
        self.ax2.set_ylim([0, np.max(sector_occupancy[0])])
        plt.show()



