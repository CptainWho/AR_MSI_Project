# -*- coding: utf-8 -*-
__author__ = 'Ecki'
__date__ = '15.07.2015'
__version__ = '1.0'

import numpy as np
from Queue import PriorityQueue
from Exercise5.util import Calculations as Calc
from copy import deepcopy

class RoomScanner:

    def __init__(self, world):
        self.cell_size = 0.1
        self.occupancy_grid = deepcopy(world.getOccupancyGrid(self.cell_size))
        self.grid = self.occupancy_grid.getGrid()

        # Iterator matrices for generator get_neighbours()
        self.adjacency_4 = [(i, j) for i in (-1, 0, 1)
                            for j in (-1, 0, 1) if not (i == -1 * j or i == j)]  # skip middle + diagonal

        # a list of all found corners
        self.found_corners = self.scan_for_corners()

        # search for the rooms
        self.rooms = self.create_room_objects(world.getRooms())

        # the last room, where robot was inside
        self.last_room = None



    def get_neighbour_values(self, grid_index):
        """
        returns the boolean values of the neighbour cells
        if cell is outside grid, the value will be False
        if there is a wall at neighbour cell it will be True
        if cell is free it will be False
        :param grid_index:
        :return: [left, down, up, right]
        """
        neighbours = []
        for [dx, dy] in self.adjacency_4:
            # Check boundaries
            if 0 <= (grid_index[0] + dx) < self.grid.shape[0] and 0 <= (grid_index[1] + dy) < self.grid.shape[1]:
                value = self.grid[grid_index[0] + dx][grid_index[1] + dy]
                neighbours.append(int(value) == 1)
            else:
                neighbours.append(False)
        return neighbours

    def get_found_corners(self):
        return self.found_corners

    def scan_for_corners(self):
        """
        finds all corners of the map
        :return:
        """
        found_corners = []

        # get all obstacle boundaries from grid
        grid = np.asarray(self.grid)
        obstacle_boundaries = np.argwhere(grid == 1)

        for grid_index in obstacle_boundaries:
            # get the neighbours in four directions
            [left, down, up, right] = self.get_neighbour_values(grid_index)
            # check neighbours to find out if cell is a croner
            if (left or right) and (up or down):
                # add cell coordinates to found corners
                x = grid_index[0] * self.cell_size
                y = grid_index[1] * self.cell_size
                found_corners.append([x, y])

        return found_corners

    def find_corners_for_room(self, room_center):
        """
        returns the 4 corners of given room center
        :param room_center:
        :return: [right_up, left_up, left_down, right_down]
        """
        [x, y] = room_center
        prio_q = PriorityQueue()
        [right_up, left_up, left_down, right_down] = [None, None, None, None]
        # find the 4 closest corners
        for corner in self.found_corners:
            dist = Calc.get_dist_from_point_to_point(room_center, corner)
            prio_q.put([dist, corner])
        for i in xrange(4):
            [x_c, y_c] = prio_q.get()[1]
            if x_c < x and y_c > y:
                left_up = [x_c, y_c]
            elif x_c < x and y_c < y:
                left_down = [x_c, y_c]
            elif x_c > x and y_c > y:
                right_up = [x_c, y_c]
            elif x_c > x and y_c < y:
                right_down = [x_c, y_c]

        return [right_up, left_up, left_down, right_down]

    def create_room_objects(self, rooms):
        """
        creates a list of all rooms
        every room becomes an object
        :param rooms:
        :return:
        """
        room_objects_list = []
        for room in rooms:
            [room_string, x, y] = room
            corners = self.find_corners_for_room([x, y])
            new_room = Room(room, corners)
            room_objects_list.append(new_room)
        return room_objects_list

    def get_room(self, robot_pos):
        """
        returns the actual room, the robot is in
        :param robot_pos:
        :return:
        """
        # check if robot is still in the room he was before
        if self.last_room is not None:
            if self.last_room.point_inside_borders(robot_pos):
                return self.last_room

        # else search if robot is in another room
        for room in self.rooms:
            if room.point_inside_borders(robot_pos):
                self.last_room = room
                return self.last_room

        # if no room was found at all return None to shwo that robot is inside corridor
        self.last_room = None
        return self.last_room


class Room:

    def __init__(self, room, corners):
        [room_name, x, y] = room
        self.center = [x, y]
        self.name = room_name
        self.corners = corners

    def get_center(self):
        return self.center

    def get_name(self):
        return self.name

    def corner_up_right(self):
        return self.corners[0]

    def corner_up_left(self):
        return self.corners[1]

    def corner_down_left(self):
        return self.corners[2]

    def corner_down_right(self):
        return self.corners[3]

    def point_inside_borders(self, point):
        """
        returns True if given poit is inside the room
        :return:
        """
        x = point[0]
        y = point[1]
        # get room borders
        top = self.corner_up_left()[1]
        left = self.corner_up_left()[0]
        right = self.corner_down_right()[0]
        bottom = self.corner_down_right()[1]
        if left < x < right and bottom < y < top:
            return True
        else:
            return False


