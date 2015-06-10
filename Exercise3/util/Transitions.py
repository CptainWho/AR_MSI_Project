# -*- coding: utf-8 -*-
""" Module Description:
Transitions for StateMachine
"""

__author1__ = 'Ecki'
__author2__ = 'Philipp Lohrer'
__date__ = '30.05.2015'
__version__ = '1.0'

# Standard library imports
import numpy as np
from Exercise2.util import Polyline
from Exercise2.util import RobotLocation
# Local imports
from Exercise2.util import Calculations as Calc




class Transitions():
    """ Class description:
    Transitions for StateMachine
    """

    def __init__(self, robot, polyline):
        """ Initialization
        :param robot: robot reference
        :param polyline: polyline reference
        :return: -
        """
        self.polyline = polyline
        self.robot = robot
        self.robot_loc = RobotLocation.RobotLocation(self.robot)
        # Tolerances to reach point / angle
        self.tol_point = 0.5
        self.tol_angle = 0.1

    def obstacle_in_sight(self):
        """ Checks whether there is a obstacle in robots line of sight
        :return: True / False
        """

        sensor_dist = np.asarray(self.robot.sense(), dtype=np.float)
        threshold = self.robot.getSize()

        # Get front sensors of the robot
        front_sensors = self.robot.getFrontSensors()

        # Calculate distance to next point -> threshold
        next_point = self.polyline.get_next_point()
        robot_point = self.robot_loc.get_robot_point()
        target_dist = Calc.get_dist_from_point_to_point(robot_point, next_point)

        # check if there is a obstacle in front of robot
        # if target is nearer than obstacle -> ignore obstacle
        if np.any(sensor_dist[front_sensors] < target_dist + threshold):
            obstacle_detected = True
        else:
            obstacle_detected = False

        return obstacle_detected

    def set_polyline_object(self, polyline):
        self.polyline = polyline

    def next_point_reached(self):
        return self.polyline.next_point_reached(self.robot_loc.get_robot_point())

    def end_point_reached(self):
        return self.polyline.end_point_reached(self.robot_loc.get_robot_point())

    def aiming_to_next_point(self):
        """ Check if robot directly looks to the next point with angle tolerance
        :return: True / False
        """
        return self.robot_loc.aiming_to_point(self.polyline.get_next_point(), self.tol_angle)
