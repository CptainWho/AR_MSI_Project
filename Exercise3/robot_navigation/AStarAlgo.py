# -*- coding: utf-8 -*-
""" Module AStarAlgo
"""

__project__ = 'Exercise 3'
__module__ = 'AStarAlgo'
__author__ = 'Philipp Lohrer'
__date__ = '16.06.2015'

__version__ = '0.1'

# Standard library imports
import numpy as np
import heapq as hq
# Local imports
from Exercise3.util import Calculations as Calc


class AStarAlgo():
    """ Class description:

    """

    def __init__(self, occupancy_grid):
        """
        :param occupancy_grid:  OccupancyGrid reference
        :return: -
        """

        self.occupancy_grid = occupancy_grid

        # Open List
        self.open_list = OpenList()
        # Closed List
        self.closed_list = ClosedList()

        # Iterator matrices for generator get_neighbours()
        self.adjacency_4 = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1) if not (i == -1 * j or i == j)]  # skip middle + diagonal
        self.adjacency_8 = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1) if not (i == j == 0)]  # skip middle

    @staticmethod
    def get_heuristic_dist(start_point, end_point):
        """ Returns heuristic (direct) distance between start- and end-point
        :param start_point: [x0, y0]
        :param end_point:   [x1, y1]
        :return: distance
        """
        return Calc.get_dist_from_point_to_point(start_point, end_point)

    def get_neighbours(self, cell, grid_size, adjacency=8):
        """ Generator: Yield all neighbours of given cell with either 4 or 8 adjacency
        :param cell:        [x,y]
        :param grid_size:   size of grid [x, y]
        :param adjacency:   amount of neighbours 4/8
        :return:            yield next neighbour
        """

        # Choose either 4 or 8 neighbours
        if adjacency == 4:
            adjacency = self.adjacency_4
        else:
            adjacency = self.adjacency_8

        for dx, dy in adjacency:
            # Check boundaries
            if 0 <= (cell[0] + dx) < grid_size[0] and 0 <= (cell[1] + dy) < grid_size[1]:
                yield (cell[0] + dx, cell[1] + dy)

    def dijkstra_algo(self, start_point, end_point):
        """ Initializes open_list with start_point and searches for shortest route to the end_point through given
        occupancy_grid via Dijkstra-Algorithm.
        :param start_point: [x0, y0]
        :param end_point:   [x1, y1]
        :return:
        """

        pass




class OpenList():
    """ Class description:
    Creates a container which acts as an open list and is internally sorted after the priority of added items
    """

    def __init__(self):
        pass

    def __len__(self):
        pass

    def __contains__(self, key):
        pass

    def __getitem__(self, key):
        pass

    def __setitem__(self, key, value):
        pass

    def __delitem__(self, key):
        pass

    def __iter__(self):
        pass


class ClosedList():
    """ Class description:
    Creates a container which acts as a closed list
    """

    def __init__(self):
        pass

    def __len__(self):
        pass

    def __contains__(self, key):
        pass

    def __getitem__(self, key):
        pass

    def __setitem__(self, key, value):
        pass

    def __delitem__(self, key):
        pass

    def __iter__(self):
        pass