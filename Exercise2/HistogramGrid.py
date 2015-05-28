# -*- coding: utf-8 -*-
""" Module HistogramGrid
Module Description:

"""

__project__ = 'Aufgabenblatt 2'
__module__  = 'HistogramGrid'
__author__  = 'Philipp Lohrer'
__email__   = 'plohrer@htwg-konstanz.de'
__date__    = '23.05.2015'

__version__ = '0.1'

# Imports
#################################################################
# Standard library imports
import numpy as np
from math import *
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
# Local imports
#################################################################


class HistogramGrid:
    """

    """

    def __init__(self, width, height, cell_size=0.1):
        """ Initialize grid
        :param width: int
        :param height: int
        :param cell_size: default 0.1
        :return:
        """

        self.x_size = int(width/cell_size)
        self.y_size = int(height/cell_size)
        # If x/y_size is even, add 1
        if self.x_size % 2 == 0:
            self.x_size += 1
        if self.y_size % 2 == 0:
            self.y_size += 1
        self.grid = np.zeros((self.x_size, self.y_size), dtype=np.int)
        self.histogram = None
        self.width = width
        self.height = height
        self.cell_size = float(cell_size)

        # print self.grid

    def move_grid(self, dx, dy):
        """ Shift grid values according to robot motion dx, dy
        :param dx: relative robot movement in x direction
        :param dy: relative robot movement in y direction
        :return: -
        """

        x_shift = int(round(dx))
        y_shift = int(round(dy))

        # Shift array values
        # np.pad(array_like,((before_axis1,after_axis1),(before_axis2,after_axis2)), mode='constant')[column_start : coulmn_stop, row_start : row_stop]

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

        print self.grid

    def set_value(self, r, theta, value=1, debug=False):
        """ Set grid value at the coordinate (x,y)
        :param r: radius
        :param theta: angle
        :param value: default 1
        :return: -
        """

        # Convert polar coordinates to cartesian coordinates
        x = r * cos(theta)
        y = r * sin(theta)

        if debug:
            print('x = %0.2f, y = %0.2f' % (x, y))

        # Coord Transformation: Place origin in middle of coord-system and reverse y-axis
        xi = int(x/self.cell_size + self.x_size / 2.0)
        yi = int(-y/self.cell_size + self.y_size / 2.0)

        if xi < 0 or xi > self.x_size:
            return
        if yi < 0 or yi > self.y_size:
            return
        self.grid[yi, xi] += value

        if debug:
            print('xi = %i, yi = %i' % (xi, yi))
            print self.grid

    def get_histogram(self, sector_angle=5, debug=False):
        """ Calculates histogram and returns the angles with corresponding occupancy values
        :param sector_angle: angle for each sector, default: 5°
        :param debug: enable/disable debug-printing
        :return: numpy array([[sector_angles],[sector_occupancy]])
        """

        # Constants
        weight_const_a = 1.0
        weight_const_b = 1.0 / 5.0  # 1 / max. sense value

        # Create arrays for sector_angles ([angles]) and sector_occupancy ([empty])
        sectors = int(360/float(sector_angle) + 1.0)  # 360° included for easier loop usage
        sector_angles = np.array([x*sector_angle for x in xrange(sectors)])
        sector_occupancy = np.zeros(sectors)
        ### Debug
        if debug:
            print('sector_angles')
            print sector_angles
            print('sector_occupancy')
            print sector_occupancy

        # Get occupied cells from histogram grid and save their indexes ( array([[y1,y2,...,yn],[x1,x2,..,xn]]) )
        occupied_cell_indexes = np.asarray(np.nonzero(self.grid))
        ### Debug
        if debug:
            print('occupied_cell_indexes')
            print occupied_cell_indexes

        # Get occupancy_values of occupied cells
        occupancy_values = self.grid[occupied_cell_indexes[0], occupied_cell_indexes[1]]
        ### Debug
        if debug:
            print('occupancy_values')
            print occupancy_values

        # Reverse Coord Transformation: Shift origin back to left upper corner of coord-system and reverse y-axis
        occupied_cell_indexes[1] = (occupied_cell_indexes[1] - (self.x_size - 1) / 2.0) * self.cell_size
        occupied_cell_indexes[0] = (-occupied_cell_indexes[0] + (self.y_size - 1) / 2.0) * self.cell_size
        ### Debug
        if debug:
            print('occupied_cell_indexes transformed')
            print occupied_cell_indexes

        # Transpose cell_indexes to get (x,y) tuples ( array([[y1,x1],[y2,x2],...[yn,xn]]) )
        occupied_cell_indexes_t = np.transpose(occupied_cell_indexes)
        ### Debug
        if debug:
            print('occupied_cell_indexes transposed')
            print occupied_cell_indexes_t

        ### Debug
        if debug:
            print('############################################')

        # Convert cell indexes to polar coordinates, calculate weight_m and add it to the corresponding sector
        # distances = array([d1,d2,...d_n])
        distances = np.sqrt(occupied_cell_indexes[0]**2 + occupied_cell_indexes[1]**2)
        # angles = array([alpha1,alpha2,...,alpha_n])
        angles = np.arctan2(occupied_cell_indexes[0], occupied_cell_indexes[1]) / pi * 180.0
        # If angle < 0 add 360° to get range 0°...360°
        angles[angles < 0] += 360.0
        # Calculate weight for each occupied cell
        weights = occupancy_values**2 * (weight_const_a - weight_const_b * distances)
        if debug:
            print 'distances'
            print distances
            print 'angles'
            print angles
            print 'occupancy values'
            print occupancy_values
            print 'weights'
            print weights

        # Add weights to corresponding sector n according to angle alpha
        print 'sector_occupancy empty'
        print sector_occupancy
        for i in xrange(len(sector_angles) - 1):
            # Find indexes of all weight-angles which fit into current sector_angle
            indexes = np.argwhere([sector_angles[i] <= angle < sector_angles[i+1] for angle in angles])
            # Add sum of all indexed weights to current sector
            sector_occupancy[i] = np.sum(weights[indexes])

        print 'sector_occupancy updated'
        print sector_occupancy

        # # OUTDATED!! Convert cell indexes to polar coordinates, get occupancy_value, calculate weight_m
        # # and add it to the corresponding sector
        # for i, cell_index in enumerate(occupied_cell_indexes_t):
        #     # Get occupancy_value of current cell
        #     occupancy_value = occupancy_values[i]
        #     # Calculate polar coordinates (distance d, angle alpha)
        #     d = sqrt(cell_index[0]**2 + cell_index[1]**2)
        #     alpha = atan2(cell_index[0], cell_index[1]) / pi * 180.0
        #
        #     ### Debug
        #     if debug:
        #         print('d: %0.2f alpha: %0.2f' % (d, alpha))
        #
        #     # Calculate weight_m
        #     weight_m = occupancy_value**2 * (weight_const_a - weight_const_b * d)
        #     ### Debug
        #     if debug:
        #         print('weight_m: %0.2f' % weight_m)

        # Generate histogram, discard 360° sector and its occupancy value (last element)
        histogram = np.array([[sector_angles[:-1]], [sector_occupancy[:-1]]])
        if debug:
            print 'histogram transposed'
            print histogram.transpose()

        self.histogram = histogram

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

    def draw_grid(self):
        """ Draw current grid
        :return: -
        """
        # plt.ion()
        plt.matshow(self.grid)
        plt.show()

    def draw_histogram(self, histogram):
        """
        :param selfself: Draw current histogram
        :return: -
        """

        # plt.ion()

        # get sector angles and occupancy
        if histogram is not None:
            sector_angles, sector_occupancy = histogram
            # Extract sector angles as strings
            text_sector_angles = [np.array_str(angle, precision=2) for angle in sector_angles[0]]
            print(sector_angles)
            # Amount of bars to be drawn (0,1...,n)
            x_ind = np.arange(np.size(sector_angles))
            # Width of the bars
            width = 0.4
            # Set up bars
            plt.bar(x_ind, sector_occupancy[0], width=width)
            # add text for labels, title and axis ticks
            plt.xlabel('sector angle')
            plt.ylabel('occupancy')
            plt.title('Histogram')
            plt.xticks(x_ind+width/2.0, text_sector_angles, rotation='vertical')

            plt.show()




#############################################################################################
# Test routines
#############################################################################################


def test_grid_generation(HG):
    HG.set_value(2,0)
    HG.set_value(2,pi/4)
    HG.set_value(3,pi/4)
    HG.set_value(2,pi/2)
    HG.set_value(2,3/4.0*pi)
    HG.set_value(4,3/4.0*pi)
    HG.set_value(2,pi)
    HG.set_value(2,5/4.0*pi)
    HG.set_value(2,3/2.0*pi)
    HG.set_value(2,7/4.0*pi)


def test_grid_move(HG):
    HG.move_grid(2, -1)


def test_get_histogram(HG):
    return HG.get_histogram(sector_angle=45, debug=True)


def test_draw_histogram(HG, histogram):
    HG.draw_histogram(histogram)


def test_draw_grid(HG):
    HG.draw_grid()


# HG = HistogramGrid(9, 9, cell_size=1.0)
# test_grid_generation(HG)
# test_draw_grid(HG)
# histogram = test_get_histogram(HG)
# test_draw_histogram(HG, histogram)