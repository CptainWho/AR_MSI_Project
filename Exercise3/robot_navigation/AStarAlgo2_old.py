# -*- coding: utf-8 -*-
__author__ = 'Admin'

from math import *
import heapq
import numpy as np
from Exercise3.util import Calculations as Calc

class AStarAlgorithm:

    def __init__(self, occupancy_grid):
        self.o_grid = occupancy_grid
        self.grid = self.o_grid.grid
        # open list
        self.end = [0, 0]
        self.open_list = OpenList()
        self.closed_list = ClosedList()
        self.knot_gap = self.o_grid.cellSize


    def heuristic_to_end(self, point):
        Calc.get_dist_from_point_to_point(point, self.end_point)

    @staticmethod
    def heuristic(start_point, end_point):
        dist = sqrt((end_point[1] - start_point[1])**2 + (end_point[0] - start_point[0])**2)
        return dist

    def match_in_grid(self, point):
        x = int(point[0]/self.o_grid.cellSize + 0.5)
        y = int(point[1]/self.o_grid.cellSize + 0.5)
        grid_pos = [x, y]
        return grid_pos

    #def calc_priority(self, point):

    def set_stat(self, grid_pos, new_status):
        self.grid[grid_pos[0]][grid_pos[1]][2] = new_status

    def update_in_grid(self, grid_pos, p, d, stat):
        self.grid[grid_pos[0]][grid_pos[1]] = [p, d, stat]

    def get_stat(self, grid_pos):
        return self.grid[grid_pos[0]][grid_pos[1]][2]

    def get_p(self, grid_pos):
        return self.grid[grid_pos[0]][grid_pos[1]][0]

    def get_d(self, grid_pos):
        return self.grid[grid_pos[0]][grid_pos[1]][1]

    def get_neighbours(self, grid_pos):
        neighbours = []
        x_pos = [grid_pos[0]-1, grid_pos[0], grid_pos[0]+1]
        y_pos = [grid_pos[1]-1, grid_pos[1], grid_pos[1]+1]

        for x in x_pos:
            for y in y_pos:
                if x == grid_pos[0] and y == grid_pos[1]:
                    pass
                else:
                    entry = self.grid[x][y]
                    if entry != 1:
                        # when not on any list yet
                        if entry < 1:
                            cost = sqrt((grid_pos[1] - y)**2 + (grid_pos[0] - x)**2)
                            neighbours.append([[x, y], cost])
                        else:
                            # when on open list
                            if self.get_stat([x, y]) == "O":
                                cost = sqrt((grid_pos[1] - y)**2 + (grid_pos[0] - x)**2)
                                neighbours.append([[x, y], cost])
        return neighbours

    def shortest_path(self, start_point, end_point, world, robot):
        """
        :param start_point:
        :param end_point:
        :param occupancy_grid:
        :return:
        """

        # save end point
        self.end = self.match_in_grid(end_point)
        start = self.match_in_grid(start_point)
        # save starting point
        self.open_list.push(self.heuristic(start, self.end), start)
        self.update_in_grid(start, start, 0, "C")

        while self.open_list.not_empty():
            v_grid_pos = self.open_list.pop()
            self.closed_list.add(v_grid_pos)
            # mark this as closed in grid
            self.set_stat(v_grid_pos, "C")
            #world.drawCircle((v_point[0], v_point[1]))
            #robot.move([0,0])
            if v_grid_pos == self.end:
                break

            # neighbours
            neighbours = self.get_neighbours(v_grid_pos)
            for neighbour in neighbours:
                # the neighbour point
                w_grid_pos = neighbour[0]
                # costs to get to this neighbour
                c_v_w = neighbour[1]
                # distance
                d_v = self.get_d(w_grid_pos)
                # calc distance, past and priority of this neighbour
                d_w = d_v + c_v_w
                p_w = self.get_p(w_grid_pos)
                h_w_z = self.heuristic(w_grid_pos, self.end)
                priority_w = d_w + h_w_z
                # if neighbour is unknown
                if self.get_stat(w_grid_pos) == 0:
                    self.open_list.push(priority_w, w_grid_pos)
                    self.set_stat(w_grid_pos, "O")
                elif d_w < self.get_d(w_grid_pos):
                    self.open_list.change_priority(w_grid_pos, priority_w)

class ClosedList:

    def __init__(self):
        self.list = []

    def set_list(self, list):
        self.list = list

    def add(self, grid_pos):
        self.list.append(grid_pos)

    def get_linked_point(self, point):
        for entry in self.list:
            linked = entry[0]
            if linked == point:
                return entry
        return None

class OpenList:

    def __init__(self):
        self.list = []
        # saves the past point
        self.past = None

    def push(self, priority, grid_pos):
        heapq.heappush(self.list, [priority, grid_pos])

    def pop(self):
        heapq.heapify(self.list)
        [priority, grid_pos] = heapq.heappop(self.list)
        return grid_pos

    def find(self, point):
        for entry in self.list:
            entry_point = entry[1]
            if entry_point == point:
                return entry
        return None

    def get_dist(self, index):
        """
        returns the distance of point at that index
        :param index:
        :return:
        """
        d = self.list[index][2]
        return d

    def change_entry(self, index, priority, d, p):
        self.list[index][0] = priority
        self.list[index][2] = d
        self.list[index][3] = p

    def change_priority(self, grid_pos, priority):
        for i in range(len(self.list)):
            if self.list[i][1] == grid_pos:
                self.list[i][0] = priority
                break

    def get_index(self, point):
        for index in range(len(self.list)):
            entry_point = self.list[index][1]
            if entry_point == point:
                return index
        return None

    def not_empty(self):
        if not self.list:
            return False
        else:
            return True
