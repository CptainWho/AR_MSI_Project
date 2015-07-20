# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
from Exercise5.navigation import AStarAlgo
from Exercise5.util import Calculations as Calc
from Exercise5.navigation import Brushfire
from datetime import datetime
from copy import deepcopy
import re

class PathScheduler:

    def __init__(self, world, occupancy_grid, start_room=None, skip_calculations=False):

        # algorithm for fast path planning
        brushfire = Brushfire.Brushfire(occupancy_grid, 0.4)
        brushfire.apply_brushfire(adjacency=4, safety_distance=0.25)
        # occupancy_grid.drawGrid()
        self.a_star_fast = AStarAlgo.AStarAlgorithm(occupancy_grid)

        self.all_rooms = world.getRooms()
        # list of all rooms to visit
        self.open_list = OpenListRooms(deepcopy(world.getRooms()))

        # set start room (here Room 01)
        if start_room == None:
            self.start_room = self.open_list[0]
        else:
            self.start_room = start_room
        self.open_list.remove(self.start_room)

        # iteration counter
        self.iteration = 0

        # the next room where robot is driving to
        self.next_room = None
        self.polyline_to_room = None
        self.next_room_center = None

        # save the last shortest length of the last calculation
        self.last_shortest_length = None

        # route algorithm
        self.shortest_route_length = None
        self.route_polyline = []

        # indicate to drive a circle at each room
        self.drive_circle = True

        # for debug saves different route lengths
        self.debug_length = []

        # for getting the next polyline
        self.poly_index = 0

        # the closed list only contains the room names
        self.closed_list = []

        # when true no calculation is done and jsut the standard route is choosen
        self.skip_calculations = skip_calculations
        self.standard_route = [[[5, 4.5], [5.25, 4.5], [5.5, 4.5], [5.75, 4.5], [6.0, 4.5], [6.25, 4.5], [6.5, 4.5], [6.75, 4.5], [7.0, 4.5], [7.25, 4.5], [7.5, 4.5], [7.75, 4.5], [8.0, 4.5], [8.25, 4.5], [8.5, 4.5], [8.75, 4.5], [9.0, 4.5]], [[9, 4.5], [8.75, 4.5], [8.5, 4.5], [8.25, 4.5], [8.0, 4.5], [7.75, 4.5], [7.5, 4.5], [7.25, 4.5], [7.0, 4.5], [6.75, 4.5], [6.5, 4.75], [6.25, 5.0], [6.0, 5.25], [5.75, 5.25], [5.5, 5.25], [5.25, 5.5], [5.0, 5.75], [5.0, 6.0], [5.0, 6.25], [5.25, 6.5], [5.5, 6.75], [5.75, 7.0], [6.0, 7.25], [6.25, 7.25], [6.5, 7.25], [6.75, 7.25], [7.0, 7.25], [7.25, 7.25], [7.5, 7.25], [7.75, 7.25], [8.0, 7.25], [8.25, 7.25], [8.5, 7.25], [8.75, 7.5], [9.0, 7.75], [9.0, 8.0], [9.0, 8.25], [9.0, 8.5], [9.0, 8.75], [9.0, 9.0], [9.0, 9.25], [9.0, 9.5]], [[9, 9.5], [9, 9.25], [9.0, 9.0], [9.0, 8.75], [9.0, 8.5], [9.0, 8.25], [9.0, 8.0], [9.0, 7.75], [8.75, 7.5], [8.5, 7.5], [8.25, 7.5], [8.0, 7.5], [7.75, 7.5], [7.5, 7.5], [7.25, 7.5], [7.0, 7.5], [6.75, 7.5], [6.5, 7.5], [6.25, 7.5], [6.0, 7.5], [5.75, 7.5], [5.5, 7.5], [5.25, 7.5], [5.0, 7.5], [4.75, 7.5], [4.5, 7.5], [4.25, 7.5], [4.0, 7.5], [3.75, 7.5], [3.5, 7.5], [3.25, 7.5], [3.0, 7.5], [2.75, 7.5], [2.5, 7.75], [2.5, 8.0], [2.5, 8.25], [2.5, 8.5], [2.5, 8.75], [2.5, 9.0], [2.5, 9.25], [2.5, 9.5], [2.5, 9.75], [2.5, 10.0], [2.5, 10.25], [2.5, 10.5], [2.5, 10.75], [2.5, 11.0], [2.5, 11.25], [2.75, 11.5], [3.0, 11.5], [3.25, 11.5], [3.5, 11.5], [3.75, 11.5], [4.0, 11.5], [4.25, 11.5], [4.5, 11.5], [4.75, 11.5], [5.0, 11.25], [5.0, 11.0], [5.0, 10.75], [5.0, 10.5], [5.0, 10.25], [5.0, 10.0], [5.0, 9.75], [5.0, 9.5]], [[5, 9.5], [5, 9.75], [5.0, 10.0], [5.0, 10.25], [5.0, 10.5], [5.0, 10.75], [5.0, 11.0], [5.0, 11.25], [5.25, 11.5], [5.5, 11.75], [5.75, 11.75], [6.0, 11.75], [6.25, 11.75], [6.5, 11.75], [6.75, 11.75], [7.0, 11.75], [7.25, 11.75], [7.5, 11.75], [7.75, 11.75], [8.0, 11.75], [8.25, 11.75], [8.5, 11.75], [8.75, 11.75], [9.0, 11.75], [9.25, 11.75], [9.5, 11.75], [9.75, 11.75], [10.0, 11.75], [10.25, 11.75], [10.5, 11.75], [10.75, 11.75], [11.0, 11.75], [11.25, 11.75], [11.5, 11.75], [11.75, 11.75], [12.0, 11.75], [12.25, 11.75], [12.5, 11.75], [12.75, 11.75], [13.0, 11.75], [13.25, 11.75], [13.5, 11.75], [13.75, 11.75], [14.0, 11.75], [14.25, 11.5], [14.5, 11.25], [14.5, 11.0], [14.5, 10.75], [14.5, 10.5], [14.5, 10.25], [14.5, 10.0], [14.5, 9.75], [14.5, 9.5], [14.5, 9.25], [14.5, 9.0]], [[14.5, 9], [14.5, 8.75], [14.5, 8.5], [14.5, 8.25], [14.5, 8.0], [14.5, 7.75], [14.5, 7.5], [14.5, 7.25], [14.5, 7.0], [14.5, 6.75], [14.5, 6.5], [14.5, 6.25], [14.5, 6.0], [14.5, 5.75], [14.5, 5.5], [14.5, 5.25], [14.5, 5.0], [14.5, 4.75], [14.5, 4.5]]]
        if self.skip_calculations:
            self.route_polyline = self.standard_route
            self.closed_list = ['Room 02', 'Room 04', 'Room 03', 'Room 05', 'Room 06']

        # create a path matrix
        room_count = len(self.all_rooms)
        self.path_matrix = PathMatrix(room_count)

        # list of all possible room combinations
        self.possible_routes = []



    def find_nearest_room(self, robot_position, open_list=None):
        """
        calculates the nearest room according to the given position
        :param robot_position:
        :return: polyline from position to room
        """
        # if no open list defined, take internal
        if open_list is None:
            open_list = self.open_list

        shortest_length = float("inf")

        for [room_string, x, y] in open_list:

            polyline = self.a_star_fast.dijkstra_algorithm(robot_position, [x, y])
            length = self.a_star_fast.get_polyline_length()


            if shortest_length > length:
                shortest_length = length
                self.polyline_to_room = polyline
                self.next_room = room_string
                self.next_room_center = [x, y]

        self.last_shortest_length = shortest_length
        return self.polyline_to_room

    def get_next_polyline(self):
        """
        only use, when find_shortest_route was called before
        returns the next polyline to next room for the robot to follow
        the polylines are returned in the order of the shortest found route
        :return:
        """
        if self.poly_index < len(self.route_polyline):
            polyline = self.route_polyline[self.poly_index]

            # update variables
            self.polyline_to_room = polyline
            self.next_room = self.closed_list[self.poly_index]
            self.next_room_center = polyline[-1]

        else:
            polyline = [self.next_room_center, self.next_room_center]

        self.poly_index += 1
        return polyline

    def find_shortest_route(self, robot_position):
        """
        searches for the shortest route through all rooms and returns a list of the resulting polylines
        :return:
        """
        if not self.skip_calculations:
        #if True:

            # 1. Calculate all possible paths
            iterations = 1
            [room_string, x, y] = self.start_room
            # make an own open list with all the rooms
            open_list = deepcopy(self.all_rooms)

            room_count = self.path_matrix.get_room_count()
            # calculate distances and polylines for all possible ways
            # i is the start room and j the target
            for i in xrange(room_count):
                for j in xrange(room_count):
                    # get the two rooms
                    [room_string_i, x_i, y_i] = open_list[i]
                    [room_string_j, x_j, y_j] = open_list[j]
                    start = [x_i, y_i]
                    end = [x_j, y_j]
                    # if same room fill in a short polyline and length = 0
                    if i == j:
                        [length, polyline] = [0, [start, end]]
                        self.path_matrix[i, j] = [length, polyline]
                    else:
                        # when reverse path exists, just copy it with reversed polyline
                        if self.path_matrix[j, i] is not None:
                            [length, polyline] = self.path_matrix[j, i]
                            r_poly = deepcopy(polyline)
                            r_poly.reverse()
                            self.path_matrix[i, j] = [length, r_poly]
                        # otherwise, calculate the path
                        else:
                            polyline = self.a_star_fast.dijkstra_algorithm(start, end)
                            print "Performing path iteration", iterations
                            length = self.a_star_fast.get_polyline_length()
                            self.path_matrix[i, j] = [length, polyline]
                            iterations += 1
                    if i == 1 and j == 0:
                        pass


            # 2. Calculate all possible routes

            # convert start room string into a number from 0 to max_rooms-1
            start_room_string = self.start_room[0]
            number = int(re.split(' ', start_room_string)[-1])
            start_room = number-1
            # start_room = 0 # room 1

            # create open list
            open_list = range(len(self.all_rooms))
            self.recursive_route_searcher(open_list, [], start_room)

            # 3. Search the route with the shortest length
            shortest_length = float("inf")
            shortest_route_polys = None
            shortest_route = None

            # go through all the calculated possible routes
            for route in self.possible_routes:
                last_room = None
                length = 0
                route_poly = []
                # go through all the rooms
                for room_no in route:
                    # skip first room
                    if last_room is None:
                        pass
                    # then calculate total lenght
                    else:
                        [length_diff, polyline] = self.path_matrix[last_room, room_no]
                        length += length_diff
                        route_poly.append(polyline)
                    last_room = room_no

                # check if actual rout is the shortest one
                if length < shortest_length:
                    shortest_length = length
                    shortest_route_polys = route_poly
                    shortest_route = route

            # 4. save the found route
            self.route_polyline = shortest_route_polys
            self.shortest_route_length = shortest_length
            self.closed_list = []
            for number in shortest_route:
                room_string = "Room 0{0}".format(number+1)
                if self.start_room[0] != room_string:
                    self.closed_list.append(room_string)


        return self.route_polyline

    def recursive_route_searcher(self, open_list, closed_list, start_room):
        """
        function that goes through all possible combinations of rooms, beginning at actual start position
        :return:
        """
        open_list.remove(start_room)
        closed_list.append(start_room)
        for room_no in open_list:
            # make a closed list to see the order of the visited rooms
            internal_closed_list = deepcopy(closed_list)
            #internal_closed_list.append(room_string)

            # remove actual room from open list
            internal_open_list = deepcopy(open_list)
            #internal_open_list.remove([room_string, x, y])

            # when there are still rooms to visit, function calls itself
            self.recursive_route_searcher(internal_open_list, internal_closed_list, room_no)


        if len(open_list) < 1:
            self.possible_routes.append(closed_list)

    def remove_from_open_list(self, room):
        [room_string, x, y] = room
        self.open_list.remove([room_string, x, y])

    def all_rooms_visited(self):
        if len(self.open_list) < 1:
            return True
        else:
            return False

    def next_room_reached(self, robot_position):
        x = self.next_room_center[0]
        y = self.next_room_center[1]

        if Calc.point_in_tol(robot_position, [x, y], 0.25):
            if not self.all_rooms_visited():
                room_string = self.next_room
                self.open_list.remove([room_string, x, y])
            return True
        else:
            return False


class PathMatrix:

    def __init__(self, room_count):
        # total number of rooms
        self.room_count = room_count
        # matrix that contains all possible paths from room i+1 to room j+1. Data is stored as [path_len, polyline]
        self.path_matrix = [[None for x in xrange(self.room_count)] for x in xrange(self.room_count)]

    def __iter__(self):
        return iter(self.path_matrix)

    def __setitem__(self, index, value):
        self.path_matrix[int(index[0])][int(index[1])] = value

    def __getitem__(self, index):
        return self.path_matrix[int(index[0])][int(index[1])]

    def get_room_count(self):
        return self.room_count


class OpenListRooms:

    def __init__(self, rooms):
        self.rooms = rooms

    def __len__(self):
        return len(self.rooms)

    def __iter__(self):
        return iter(self.rooms)

    def __getitem__(self, index):
        return self.rooms[index]

    def get_all_rooms(self):
        return self.rooms

    def remove(self, room):
        [room_string, x, y] = room
        self.rooms.remove([room_string, x, y])




# OBSOLETE
# def recursive_funct_backup(self, open_list):
#     rooms = []
#     for [room_string, x, y] in open_list:
#         internal_open_list = copy.deepcopy(open_list)
#         internal_open_list.remove([room_string, x, y])
#         next_rec = self.recursive_route_searcher(internal_open_list)
#         if len(next_rec) > 0:
#             rooms.append([room_string, next_rec])
#         else:
#             rooms.append(room_string)
#
#     return rooms
#
# class Route:
#
#     def __init__(self):
#         pass

# def find_shortest_route(self, robot_position):
#         """
#         searches for the shortest route through all rooms and returns a list of the resulting polylines
#         :return:
#         """
#         if not self.skip_calculations:
#             [room_string, x, y] = self.start_room
#             #open_list = self.open_list[0:2]
#             open_list = self.open_list
#             self.recursive_route_searcher(open_list, [x, y], 0, [], [])
#
#         #print self.route_polyline
#         #print self.closed_list
#
#         return self.route_polyline
#
#     def recursive_route_searcher(self, open_list, start_point, length, polylines, closed_list):
#         """
#         function that goes through all possible combinations of rooms, beginning at actual start position
#         :return:
#         """
#         for [room_string, x, y] in open_list:
#             # make a closed list to see the order of the visited rooms
#             internal_closed_list = copy.deepcopy(closed_list)
#             internal_closed_list.append(room_string)
#
#             # remove actual room from open list
#             internal_open_list = copy.deepcopy(open_list)
#             internal_open_list.remove([room_string, x, y])
#
#             # the polylines that where found
#             internal_polylines = copy.deepcopy(polylines)
#             t_start = datetime.now()
#             poly = self.a_star_fast.dijkstra_algorithm(start_point, [x, y])
#             print self.iteration
#             self.iteration += 1
#             t_delta = (datetime.now() - t_start).total_seconds()
#             print internal_closed_list
#             print 'Astar in: %0.5f seconds' % t_delta
#
#
#             internal_polylines.append(poly)
#             internal_length = length + self.a_star_fast.last_poly_length
#
#             # for debug
#             if internal_closed_list == ['Room 02', 'Room 04', 'Room 05', 'Room 06', 'Room 03'] or \
#                             internal_closed_list == ['Room 02', 'Room 06', 'Room 05', 'Room 04', 'Room 03'] or \
#                             internal_closed_list == ['Room 02', 'Room 06', 'Room 05', 'Room 03', 'Room 04']:
#                 self.debug_length.append(internal_length)
#
#             if len(internal_open_list) > 0:
#                 # when there are still rooms to visit, function calls itself
#                 self.recursive_route_searcher(internal_open_list, [x, y], internal_length, internal_polylines, internal_closed_list)
#             else:
#                 # when finished with this iteration, look if the actual route is smaller, than the shortest route found before
#                 if self.shortest_route_length is None or self.shortest_route_length > internal_length:
#                     self.shortest_route_length = internal_length
#                     self.closed_list = internal_closed_list
#                     self.route_polyline = internal_polylines
