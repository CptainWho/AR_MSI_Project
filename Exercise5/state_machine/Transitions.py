# -*- coding: utf-8 -*-
""" Module Description:
Transitions for StateMachine
"""

__author1__ = 'Ecki'
__author2__ = 'Philipp Lohrer'
__date__ = '16.07.2015'
__version__ = '1.1'

# Changelog:
# 16.07.2015 (Phil):    updated imports, improved obstacle_in_sight

# Standard library imports
import numpy as np
# Local imports
from Exercise5.util import Calculations as Calc


class Transitions:
    """ Class description:
    Transitions for StateMachine
    """

    def __init__(self, robot, robot_loc, carrot_donkey, path_sched, room_scan):
        """ Initialization
        :param robot: robot reference
        :param polyline: polyline reference
        :return: -
        """
        self.carrot_donkey = carrot_donkey
        self.path_sched = path_sched
        self.room_scan = room_scan
        self.robot = robot
        self.robot_loc = robot_loc
        # Tolerances to reach point / angle
        self.tol_point = 0.5
        self.tol_angle = 0.1

    def next_room_reached(self):
        robot_position = self.robot_loc.get_robot_point()
        return self.path_sched.next_room_reached(robot_position)

    def all_rooms_visited(self):
        return self.path_sched.all_rooms_visited()

    def all_corners_inspected(self):
        """
        returns True if robot has look into every corner of the room
        :return:
        """
        return self.room_scan.all_corners_inspected()

    def aiming_to_carrot(self):
        """
        checks if robot is aiming to carrot
        :return: True/False
        """
        carrot = self.carrot_donkey.get_carrot_position()
        if self.robot_loc.aiming_to_point(carrot, self.tol_angle):
            return True
        else:
            return False

    def obstacle_in_sight(self):
        """ Checks whether there is a obstacle in robots line of sight
        :return: True / False
        """

        sensor_dist = np.asarray(self.robot.sense(), dtype=np.float)

        threshold = 1.0 * self.robot.getSize()

        # Get front sensors of the robot
        front_sensors = self.robot.getFrontSensors()

        # Calculate distance to next point -> threshold
        # next_point = self.carrot_donkey.get_next_point()
        next_point = self.carrot_donkey.get_carrot_position()
        robot_point = self.robot_loc.get_robot_point()
        target_dist = Calc.get_dist_from_point_to_point(robot_point, next_point)

        # check if there is a obstacle in front of robot
        # if target is nearer than obstacle -> ignore obstacle
        sensor_dist_min = np.nanmin(sensor_dist[front_sensors])
        if sensor_dist_min < (target_dist + threshold):
            obstacle_detected = True
        else:
            obstacle_detected = False

        return obstacle_detected

