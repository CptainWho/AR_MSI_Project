# -*- coding: utf-8 -*-
__author__ = 'Admin'

from math import *
import heapq
import numpy as np
from Exercise3.util import Calculations as Calc

class AStarAlgorithm:

    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        # open list
        self.end_point = [0, 0]
        self.open_list = OpenList()
        self.closed_list = ClosedList()
        self.knot_gap = self.grid.cellSize

    def heuristic_to_end(self, point):
        Calc.get_dist_from_point_to_point(point, self.end_point)

    @staticmethod
    def heuristic(start_point, end_point):
        dist = sqrt((end_point[1] - start_point[1])**2 + (end_point[0] - start_point[0])**2)
        return dist

    def match_in_grid(self, point):
        x = int(point[0]/self.grid.cellSize + 0.5) * self.grid.cellSize
        y = int(point[1]/self.grid.cellSize + 0.5) * self.grid.cellSize
        return [x, y]

    #def calc_priority(self, point):

    def get_neighbours(self, point):
        """
        returns all neighbours of a point as long as they are not blocked
        :param point:
        :return: a list with the entrys [point, cost] for every neighbour
        """
        x_pos = [-self.knot_gap, 0, self.knot_gap]
        y_pos = [-self.knot_gap, 0, self.knot_gap]
        neighbours = []

        for del_x in x_pos:
            for del_y in y_pos:
                if del_x == 0 and del_y == 0:
                    # skip in middle
                    pass
                else:
                    x = point[0] + del_x
                    y = point[1] + del_y
                    if self.grid.getValue(x, y) < 1:
                        cost = sqrt(del_x**2 + del_y**2)
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
        self.end_point = self.match_in_grid(end_point)
        # save starting point
        self.open_list.push(start_point, self.heuristic_to_end(start_point), 0, start_point)

        while self.open_list.not_empty():
            [v_point, d_v, p_v] = self.open_list.pop()
            self.closed_list.add(v_point, p_v)
            #world.drawCircle((v_point[0], v_point[1]))
            #robot.move([0,0])
            if v_point == end_point:
                break

            # neighbours
            neighbours = self.get_neighbours(v_point)
            neighbours = self.closed_list.filter_existing_neighbours(neighbours)
            for neighbour in neighbours:
                # the neighbour point
                w_point = neighbour[0]
                # costs to get to this neighbour
                c_v_w = neighbour[1]
                # search for neighbours in open list
                index = self.open_list.get_index(w_point)
                # calc distance, past and priority of this neighbour
                d_w = d_v + c_v_w
                p_w = v_point
                h_w_z = self.heuristic(w_point, end_point)
                priority_w = d_w + h_w_z
                # if neighbour is unknown
                if index is None:
                    self.open_list.push(w_point, priority_w, d_w, p_w)
                elif d_w < self.open_list.get_dist(index):
                    self.open_list.change_entry(index, priority_w, d_w, p_w)

class ClosedList:

    def __init__(self):
        self.list = []

    def set_list(self, list):
        self.list = list

    def add(self, point, p):
        self.list.append([point, p])

    def get_linked_point(self, point):
        for entry in self.list:
            linked = entry[0]
            if linked == point:
                return entry
        return None

    def filter_existing_neighbours(self, neighbours):
        filtered_list = [i for i in neighbours if i[0] not in self.list[0]]
        return filtered_list

class OpenList:

    def __init__(self):
        self.list = []
        # saves the past point
        self.past = None

    def push(self, point, priority, distance, past):
        p = past
        d = distance
        heapq.heappush(self.list, [priority, point, d, p])

    def pop(self):
        heapq.heapify(self.list)
        [priority, point, d, p] = heapq.heappop(self.list)
        return [point, d, p]

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
