# -*- coding: utf-8 -*-
""" Module HistogramGrid
Module Description:

"""

__project__ = 'Aufgabenblatt 2'
__module__  = 'HistogramGrid'
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
#################################################################


class HistogramGrid:
    '''

    '''

    def __init__(self, width, height, cell_size=0.1):
        """ Initialize grid
        :param width: int
        :param height: int
        :param cell_size: default 0.1
        :return:
        """

        self.x_size = int(width/cell_size)
        self.y_size = int(height/cell_size)
        self.grid = np.zeros((self.x_size, self.y_size), dtype=np.int)
        # self.grid = np.random.randint(0, 100, (self.x_size, self.y_size))
        self.width = width
        self.height = height
        self.cell_size = float(cell_size)

        # print self.grid

    def move_grid(self, dx, dy):
        """ Shift grid values according to dx, dy
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

    def set_value(self, r, theta, value=1):
        """ Set grid value at the coordinate (x,y)
        :param x: coord x
        :param y: coord y
        :param value: default 1
        :return: -
        """

        # Convert polar coordinates to cartesian coordinates
        x = r * cos(theta)
        y = r * sin(theta)

        print('x = %i, y = %i' % (x, y))

        if x < 0 or x > self.width:
            return
        if y < 0 or y > self.width:
            return
        xi = int(x/self.cell_size)
        yi = int(y/self.cell_size)
        self.grid[xi, yi] = value

        print('xi = %i, yi = %i' % (xi, yi))

        print self.grid

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
        return self.grid[xi, yi]








HG = HistogramGrid(0.5, 0.5)
HG.set_value(0.4,pi/4)

# print('dx = 2, dy = -1')
# HG.move_grid(2,-1)

# print('dx = 1')
# HG.move_grid(1,0)
# print('dx = -1')
# HG.move_grid(-1,0)
#
# print('dy = 1')
# HG.move_grid(0,1)
# print('dy = -1')
# HG.move_grid(0,-1)























