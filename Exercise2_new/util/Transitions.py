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
        self.tol_point = 0.5
        self.tol_angle = 0.1

    def obstacle_in_sight(self):
        """ Checks whether there is a obstacle in robots line of sight
        :return: True / False
        """

        sensor_dist = np.asarray(self.robot.sense())
        threshold = self.robot.getMaxSenseValue()/2.0

        # check if there is a obstacle in front of robot
        # (directions: 9 is angle = 0)
        for i in sensor_dist[3:15]:
            if i < threshold and i is not None:
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
        return self.robot_loc.aiming_to_point(self.polyline.get_next_point(), self.tol_angle)
