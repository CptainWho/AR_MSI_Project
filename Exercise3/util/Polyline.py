__author__ = 'Ecki'

from math import *
from Exercise2.util import Calculations as Calc

class Polyline:

    def __init__(self, polyline):
        self.polyline = polyline
        self.tol_point = 0.1

    def set_polyline(self, polyline):
        """
        defines the polyline for robot to follow
        :param polyline:
        :return:
        """
        self.polyline = polyline

    def set_point_tolerance(self, tol):
        self.tol_point = tol

    def get_point_tolerance(self):
        return self.tol_point

    def get_next_point(self):
        return self.polyline[1]

    def get_last_point(self):
        return self.polyline[0]

    def next_point_reached(self, robot_pos):
        """
        check if robot has reached a point and updates to next point
        :param robot_pos: position of robot
        :return:
        """
        if Calc.point_in_tol(robot_pos, self.get_next_point(), self.get_point_tolerance()):
            if len(self.polyline) > 2:
                self.polyline.remove(self.polyline[0])
            return True
        else:
            return False

    def end_point_reached(self, robot_pos):
        """
        check if End Point is reached
        :param robot_pos: position of robot
        :return: True/False
        """
        if len(self.polyline) == 2:
            if Calc.point_in_tol(robot_pos, self.get_next_point(), self.get_point_tolerance()):
                return True
            else:
                return False
        else:
            return False
