# -*- coding: utf-8 -*-
""" Module Brushfire
"""

__project__ = 'Exercise 3'
__module__  = 'E3_A1'
__author__  = 'Philipp Lohrer'
__date__    = '11.06.2015'

__version__ = '0.1'

# Standard library imports
import numpy as np
import heapq as hq
from math import pi
# Local imports
from HTWG_Robot_Simulator_V1 import (emptyWorld, Robot)


def apply_brushfire(grid, robot, connectivity=8, safety_distance=None):
    """
    :param grid: occupancy_grid
    :param robot: robot reference
    :param connectivity: amount neighbors 4/8
    :param safety_distance:
    :return: occupancy_grid
    """

    robot_radius = robot.getSize()
    if safety_distance is None:
        safety_distance = robot_radius
    if connectivity != 4 and connectivity != 8:
        connectivity = 8

    # 1. Get all occupied cell indexes
    occupied_cells = np.argwhere(grid == 1)


