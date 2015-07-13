# -*- coding: utf-8 -*-
""" Module Brushfire
"""

__project__ = 'Exercise 3'
__module__  = 'E3_A1'
__author__  = 'Philipp Lohrer'
__date__    = '16.06.2015'

__version__ = '1.0'

# Standard library imports
import numpy as np
# Local imports


class Brushfire():
    """ Class description:

    """

    def __init__(self, occupancy_grid, robot_radius):
        """ Initialize Brushfire-Algorithm
        :param occupancy_grid:  OccupancyGrid reference
        :param robot:           robot reference
        :return: -
        """

        self.occupancy_grid = occupancy_grid
        self.robot_radius = robot_radius

        # Iterator matrices for generator get_neighbours()
        self.adjacency_4 = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1) if not (i == -1 * j or i == j)]  # skip middle + diagonal
        self.adjacency_8 = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1) if not (i == j == 0)]  # skip middle

    def apply_brushfire(self, occupancy_grid=None, adjacency=8, safety_distance=None):
        """ Applies brushfire-algorithm to given occupancy-grid and updates it in place
        occupancy values (float) between 1 (obstacle) to 0 (no obstacle)
        :param occupancy_grid:  (optional) OccupancyGrid reference
        :param adjacency:       (optional) amount neighbors 4/8
        :param safety_distance: (optional) safety distance to wall, i.e. 5
        :return:                -
        """

        if occupancy_grid is None:
            occupancy_grid = self.occupancy_grid

        cell_size = occupancy_grid.getCellSize()
        grid_size = occupancy_grid.getGridSize()
        brushfire_grid = occupancy_grid.getGrid()

        if safety_distance is None:
            safety_distance = self.robot_radius
        if adjacency != 4 and adjacency != 8:
            adjacency = 8

        # 1. Get all obstacle boundaries (= occupied cells from occupancy_grid)
        obstacle_boundaries = np.argwhere(brushfire_grid == 1)

        # 2. Define dictionary open_list and initialize it with obstacle_boundaries (set distance d to 0)
        open_list = dict([(tuple(cell), 0) for cell in obstacle_boundaries])
        closed_list = {}

        # 3. Iterate through open_list until all cells are visited
        while open_list:  # True as long dictionary contains items!
            # 3.1 Get an occupied cell from open_list
            cell, d_cell = open_list.popitem()
            # 3.2 Get all neighbours
            for neighbour in self.get_neighbours(cell, grid_size, adjacency):
                if neighbour not in open_list and neighbour not in closed_list:
                    # Neighbour is unknown -> add it to open_list
                    open_list[neighbour] = d_cell + cell_size
                elif neighbour in open_list and d_cell + cell_size < open_list[neighbour]:
                    open_list[neighbour] = d_cell + cell_size
                elif neighbour in closed_list and d_cell + cell_size < closed_list[neighbour]:
                    closed_list[neighbour] = d_cell + cell_size
            # 3.3 Add current cell to closed_list
            closed_list[cell] = d_cell

        # 4. Update occupancy grid and scale occupancy values according to distance to nearest obstacle
        for cell in closed_list:
            if closed_list[cell] < (self.robot_radius + safety_distance):
                # Calculate weight of the cell according to distance to nearest obstacle
                occupancy_value = 1 - (closed_list[cell] / float(self.robot_radius + safety_distance))
            else:
                # Cell is far enough away from obstacle -> zero occupancy
                occupancy_value = 0
            occupancy_grid.set_value_at_index(cell[0], cell[1], occupancy_value)

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




































