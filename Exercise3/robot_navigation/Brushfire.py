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
from Queue import Queue
from math import pi
# Local imports
from HTWG_Robot_Simulator_V1 import (emptyWorld, Robot)


def apply_brushfire(occupancy_grid, robot, adjacency=8, safety_distance=None):
    """
    :param occupancy_grid: occupancy_grid
    :param robot: robot reference
    :param adjacency: amount neighbors 4/8
    :param safety_distance:
    :return: updated occupancy_grid
    """

    robot_radius = robot.getSize()
    cell_size = occupancy_grid.getCellSize()
    grid = np.asarray(occupancy_grid.getGrid())

    if safety_distance is None:
        safety_distance = robot_radius
    if adjacency != 4 and adjacency != 8:
        adjacency = 8

    # 1. Get all obstacle boundaries (= occupied cells from occupancy_grid)
    obstacle_boundaries = np.argwhere(grid == 1)

    # 2. Define open_list and initialize it with obstacle_boundaries (set distance d to 0)
    open_list = Queue()
    for cell in obstacle_boundaries:
        open_list.put((0, cell))

    # 3. Iterate through open_list until all cells are visited
    while not open_list.empty():
        # 3.1 Get an occupied cell from open_list
        d, cell = open_list.get()
        # 3.2 Get all neighbours
        print 'Neighbours of cell [%i, %i]' % tuple(cell)
        neighbours = get_neighbours(occupancy_grid, cell, adjacency)
        for neighbour in neighbours:
            print '\t' + str(neighbour)


adjacency_4 = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1) if not (i == -1 * j or i == j)]  # skip middle + diagonal
adjacency_8 = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1) if not (i == j == 0)]  # skip middle


def get_neighbours(occupancy_grid, cell, adjacency):
    """ Return all neighbours of given cell
    :param occupancy_grid: occupancy_grid reference
    :param cell: [x,y]
    :param adjacency: amount of neighbours 4/8
    :return: yield next neighbour
    """

    # Choose either 4 or 8 neighbours
    if adjacency == 4:
        adjacency = adjacency_4
    else:
        adjacency = adjacency_8

    cell_x, cell_y = cell
    grid_size_x, grid_size_y = occupancy_grid.getGridSize()

    for dx, dy in adjacency:
        # Check boundaries
        if 0 <= (cell_x + dx) < grid_size_x and 0 <= (cell_y + dy) < grid_size_y:
            yield [cell_x + dx, cell_y + dy]





































