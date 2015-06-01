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

        self.robot = robot
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

    def next_point_reached(self):
        """ Check if robot has reached a point and updates to next point
        :return: True / False
        """

        if self.indsideTol(self.getNextPoint(), self.tolPoint):
            if len(self.polyline) > 2:
                self.polyline.remove(self.polyline[0])
            return True
        else:
            return False

    def endpoint_reached(self):
        """ Check if End Point has been reached
        :return: True / False
        """

        if len(self.polyline) == 2:
            if self.indsideTol(self.getNextPoint(), self.tolPoint):
                return True
            else:
                return False
        else:
            return False

    def aiming_to_next_point(self):
        """ Check if robot directly looks to the next point with angle tolerance
        :return: True / False
        """

        if self.insideAngleTol(self.getAngleToPoint(self.getNextPoint()), self.tolAngle):
            return True
        else:
            return False