__author__ = 'Ecki'

from math import *
from Exercise2_new.util import Calculations as Calc

class RobotLocation:

    def __init__(self, robot):
        self.robot = robot

    def get_robot_position(self):
        """
        :return: the robots position
        """
        return  self.robot.getTrueRobotPose()

    def get_robot_point(self):
        """
        :return: robots x and y value in global coordinate system
        """
        [x, y, theta] = self.get_robot_position()
        return [x, y]

    def get_robot_angle(self):
        """
        :return: robots angle in global coordinate system
        """
        [x, y, theta] = self.get_robot_position()
        return theta

    def robot_inside_tolerance(self, p_target, tolerance):
        """
        check whether robot in within tolerance of target point
        :param p_target: the target point [x, y]
        :param tolerance: tolaerance radius around the point
        :return: True or False
        """
        p = self.get_robot_point()
        return Calc.point_in_tol(p, p_target, tolerance)
