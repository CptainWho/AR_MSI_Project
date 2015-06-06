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
from Exercise2_new.util import Polyline
from Exercise2_new.util import RobotLocation
# Local imports
from Exercise2_new.util import Calculations as Calc



class Transitions():
    """ Class description:
    Transitions for StateMachine
    """

    def __init__(self, robot):
        """ Initialization
        :param robot: robot reference
        :return: -
        """
        self.polyline = None
        self.robot = robot
        self.robot_loc = RobotLocation.RobotLocation(self.robot)
        # Tolerances to reach point / angle
        self.tol_point = 0.2
        self.tol_angle = 0.1

    def obstacle_in_sight(self):
        """ Checks whether there is a obstacle in robots line of sight
        :return: True / False
        """

        sensor_dist = self.robot.sense()
        # check if there is a obstacle in front of robot
        if np.any(sensor_dist):
            return True
        return False

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
        return self.robot_loc.inside_angle_tol(self.polyline.get_next_point(), self.tol_angle)
