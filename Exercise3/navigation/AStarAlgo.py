# -*- coding: utf-8 -*-
""" Module AStarAlgo
"""

__project__ = 'Exercise 3'
__module__ = 'AStarAlgo'
__author__ = 'Philipp Lohrer'
__date__ = '19.06.2015'

__version__ = '1.0'

# Standard library imports
from math import sqrt
import numpy as np
import heapq as hq
# Local imports
from Exercise3.util import Calculations as Calc


class AStarAlgorithm():
    """ Class description:

    """

    def __init__(self, occupancy_grid):
        """
        :param occupancy_grid:  OccupancyGrid reference
        :return: -
        """

        self.grid_ref = occupancy_grid
        self.grid = self.grid_ref.getGrid()
        self.cell_size = self.grid_ref.getCellSize()
        self.grid_size = self.grid_ref.getGridSize()
        self.grid_width = self.grid_ref.getGridWidth()
        self.grid_height = self.grid_ref.getGridHeight()
        self.end_point = None
        self.start_point = None

        # Open List
        self.open_list = OpenList()
        # Closed List
        self.closed_list = ClosedList(self.grid_size[0], self.grid_size[1])

        # Iterator matrices for generator get_neighbours()
        self.adjacency_4 = [(i, j) for i in (-self.cell_size, 0, self.cell_size)
                            for j in (-self.cell_size, 0, self.cell_size) if not (i == -1 * j or i == j)]  # skip middle + diagonal
        self.adjacency_8 = [(i, j) for i in (-self.cell_size, 0, self.cell_size)
                            for j in (-self.cell_size, 0, self.cell_size) if not (i == j == 0)]  # skip middle


    def get_heuristic_dist_to_end(self, point):
        """ Returns heuristic (direct) distance between start- and end-point
        :param point:   [x, y]
        :return:        distance
        """

        if self.end_point is not None:
            return Calc.get_dist_from_point_to_point(point, self.end_point)

    def match_in_grid(self, point):
        """ Transforms coordinate of given point to resulting grid index
        :param point:   [x, y]
        :return:        [index_x, index_y]
        """

        index_x = int(point[0]/self.cell_size + 0.5)
        index_y = int(point[1]/self.cell_size + 0.5)
        return [index_x, index_y]

    def get_neighbours(self, cell, adjacency=8):
        """ Generator: Yield all neighbours of given cell with either 4 or 8 adjacency
        :param cell:        [x,y]
        :param adjacency:   amount of neighbours 4/8
        :return:            yield next neighbour[point[x, y], cost, occupancy]
        """

        # Choose either 4 or 8 neighbours
        if adjacency == 4:
            adjacency = self.adjacency_4
        else:
            adjacency = self.adjacency_8

        for dx, dy in adjacency:
            # Check boundaries
            point = [cell[0] + dx, cell[1] + dy]
            if 0 <= point[0] < self.grid_width and 0 <= point[1] < self.grid_height:
                # Calculate cost to get to this neighbour and get occupancy from grid
                cost = sqrt(dx**2 + dy**2)
                occupancy = self.grid_ref.getValue(point[0], point[1])
                # Yield neighbour point, cost and occupancy
                yield (point, cost, occupancy)

    def create_polyline(self, last_point):
        """ Create polyline starting at end_point until arriving at start_point via saved predecessors
        :param last_point:  [x, y]
        :return:            polyline
        """

        polyline = []
        predecessor = self.closed_list[self.match_in_grid(last_point)]
        while predecessor != self.start_point:
            polyline.append(predecessor)
            predecessor = self.closed_list[self.match_in_grid(predecessor)]
        polyline.append(self.start_point)
        polyline.reverse()

        return polyline

    def dijkstra_algorithm(self, start_point, end_point, adjacency=8):
        """ Initializes open_list with start_point and searches for shortest route to the end_point through given
        occupancy_grid via Dijkstra-Algorithm.
        :param start_point: [x0, y0]
        :param end_point:   [x1, y1]
        :return:            polyline for shortest path
        """

        # Save end and start point
        self.end_point = end_point
        self.start_point = start_point
        # Initialize dijkstra-algorithm with start_point
        self.open_list.push(self.start_point, self.get_heuristic_dist_to_end(self.start_point), 0, self.start_point)

        while self.open_list.not_empty():
            # Get Point with highest priority (= lowest distance)
            v_priority, v_point, d_v, p_v = self.open_list.pop()
            # Add Point to closed_list
            v_point_index = self.match_in_grid(v_point)
            self.closed_list[v_point_index] = p_v

            # If current point is the end_point: stop algorithm and create polyline
            if v_point == self.end_point:
                polyline = self.create_polyline(v_point)
                return polyline

            # Point is not end_point -> get neighbouring points
            for neighbour in self.get_neighbours(v_point, adjacency):
                # The neighbour point
                w_point = neighbour[0]
                w_point_index = self.match_in_grid(w_point)
                # Occupancy at current point
                w_point_occupancy = neighbour[2]
                # If w_point is already in closed_list or if it's occupied by an obstacle: skip point_w
                if self.closed_list[w_point_index] is None and w_point_occupancy < 1:
                    # Costs to get to this neighbour
                    c_v_w = neighbour[1]
                    # Calculate new distance to end_point, new priority h_w_z and set predecessor
                    d_w = d_v + c_v_w
                    h_w_z = self.get_heuristic_dist_to_end(w_point)
                    p_w = v_point
                    # Scale priority according to occupancy ([0..1]) at grid[point] -> priority * [1.0 .. 2.0]
                    priority_w = (d_w + h_w_z) * (w_point_occupancy + 1)
                    # Check if neighbour is already in open_list
                    if w_point not in self.open_list:
                        # Neighbour is still unknown -> add to open_list
                        self.open_list.push(w_point, priority_w, d_w, p_w)
                    else:
                        # Neighbour is already in open_list, check if update is necessary
                        if d_w < self.open_list.get_dist(w_point):
                            self.open_list.update_entry(w_point, priority_w, d_w, p_w)


class OpenList():
    """ Class description:
    Creates a container which acts as an open list and is internally sorted after the priority of added items
    """

    def __init__(self, values=None):
        if values is None:
            self.values = []
        else:
            self.values = values

    def __len__(self):
        return len(self.values)

    def __contains__(self, point):
        return any(entry[1] == point for entry in self.values)

    def __iter__(self):
        return iter(self.values)

    def pop(self):
        hq.heapify(self.values)
        return hq.heappop(self.values)

    def push(self, point, priority, distance, predecessor):
        hq.heappush(self.values, [priority, point, distance, predecessor])

    def find_entry(self, point):
        for entry in self.values:
            if entry[1] == point:
                return self.values.index(entry)

    def get_dist(self, point):
        index = self.find_entry(point)
        return self.values[index][2]

    def update_entry(self, point, priority, distance, predecessor):
        index = self.find_entry(point)
        self.values[index][0] = priority
        self.values[index][2] = distance
        self.values[index][3] = predecessor

    def not_empty(self):
        return True if self.values else False


class ClosedList():
    """ Class description:
    Creates a container which acts as a closed list
    """

    def __init__(self, grid_width, grid_height):
        self.closed_map = [[None for x in xrange(grid_height)] for x in xrange(grid_width)]

    def __len__(self):
        return np.size(self.closed_map)

    def __getitem__(self, point):
        return self.closed_map[int(point[0])][int(point[1])]

    def __setitem__(self, point, value):
        self.closed_map[int(point[0])][int(point[1])] = value

    def __contains__(self, key):
        return True if self.closed_map[int(key[0])][int(key[1])] is not None else False

    def __iter__(self):
        return iter(self.closed_map)

