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

    def __init__(self, robot, carrot_donkey, path_sched):
        """ Initialization
        :param robot: robot reference
        :param polyline: polyline reference
        :return: -
        """
        self.carrot_donkey = carrot_donkey
        self.path_sched = path_sched
        self.robot = robot
        self.robot_loc = RobotLocation.RobotLocation(self.robot)
        # Tolerances to reach point / angle
        self.tol_point = 0.5
        self.tol_angle = 0.1

    def next_room_reached(self):
        robot_position = self.robot_loc.get_robot_point()
        return self.path_sched.next_room_reached(robot_position)

    def obstacle_in_sight(self):
        """ Checks whether there is a obstacle in robots line of sight
        :return: True / False
        """

        sensor_dist = np.asarray(self.robot.sense(), dtype=np.float)
        threshold = self.robot.getSize()*2.0

        # Get front sensors of the robot
        front_sensors = self.robot.getFrontSensors()

        # Calculate distance to next point -> threshold
        next_point = self.carrot_donkey.get_next_point()
        robot_point = self.robot_loc.get_robot_point()
        target_dist = Calc.get_dist_from_point_to_point(robot_point, next_point)

        # check if there is a obstacle in front of robot
        # if target is nearer than obstacle -> ignore obstacle
        if np.any((sensor_dist[front_sensors] * 2.0) < target_dist + threshold):
            obstacle_detected = True
        else:
            obstacle_detected = False

        return obstacle_detected


