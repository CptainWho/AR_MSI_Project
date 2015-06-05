__author__ = 'Ecki'

from math import *
from Exercise2_new.util import Calculations as Calc

class RobotLocation:

    def __init__(self, robot):
        self.robot = robot
        self.angle_tol = 10*pi/180

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

    def get_positive_robot_angle(self):
        """
        :return: robots positive angle in global coordinate system
        """
        theta = self.get_robot_angle() % (2 * pi)
        return theta

    def inside_angle_tol(self, point, tolerance):
        """
        checks if robot is aming to a point
        :return: True/False
        """
        return Calc.angle_in_tol(self.get_robot_angle(),
                          Calc.get_angle_from_robot_to_point(self.get_robot_point(), point), tolerance)

    def robot_inside_tolerance(self, p_target, tolerance):
        """
        check whether robot in within tolerance of target point
        :param p_target: the target point [x, y]
        :param tolerance: tolaerance radius around the point
        :return: True or False
        """
        p = self.get_robot_point()
        return Calc.point_in_tol(p, p_target, tolerance)

    def get_angle_from_robot_to_point(self, point):
        return Calc.get_angle_from_robot_to_point(self.robot, point)
