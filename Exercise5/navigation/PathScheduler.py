# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
from Exercise5.navigation import AStarAlgo
from Exercise5.util import Calculations as Calc
from Exercise5.navigation import Brushfire

class PathScheduler:

    def __init__(self, world):
        self.world = world

        # algorithm for fine path planning
        self.occupancy_grid = world.getOccupancyGrid(0.25)
        brushfire = Brushfire.Brushfire(self.occupancy_grid, 0.4)
        brushfire.apply_brushfire(adjacency=8, safety_distance=0.25)
        self.a_star = AStarAlgo.AStarAlgorithm(self.occupancy_grid)

        # algorithm for fast path planning
        self.occupancy_grid_rough  = world.getOccupancyGrid(0.5)
        self.a_star_fast = AStarAlgo.AStarAlgorithm(self.occupancy_grid_rough)

        # list of all rooms to visit
        self.open_list = world.getRooms()
        #self.remove_from_open_list(self.open_list[0])
        # list of all visited rooms
        self.closed_list = []

        # the next room where robot is driving to
        self.next_room = None
        self.polyline_to_room = None
        self.next_room_center = None

    def find_nearest_room(self, robot_position):
        shortest_lenght = float("inf")

        for [room_string, x, y] in self.open_list:
            polyline = self.a_star_fast.dijkstra_algorithm(robot_position, [x, y])
            length = self.a_star_fast.get_polyline_length()
            if shortest_lenght > length:
                shortest_lenght = length
                self.polyline_to_room = polyline
                self.next_room = room_string
                self.next_room_center = [x, y]

        return self.polyline_to_room

    def remove_from_open_list(self, room):
        [room_string, x, y] = room
        self.open_list.remove([room_string, x, y])

    def add_to_closed_list(self, room):
        [room_string, x, y] = room
        self.closed_list.append([room_string, x, y])

    def next_room_reached(self, robot_position):
        x = self.next_room_center[0]
        y = self.next_room_center[1]

        if Calc.point_in_tol(robot_position, [x, y], 0.1):
            room_string = self.next_room
            self.remove_from_open_list([room_string, x, y])
            self.add_to_closed_list([room_string, x, y])
            return True
        else:
            return False



