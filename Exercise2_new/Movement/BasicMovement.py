""" Module Description:
Contains methods for basic robot movement.
Class BasicMovement has to be initialized first.
Afterwards all methods can be directly accessed.
"""

__author1__ = 'Ecki'
__author2__ = 'Philipp Lohrer'
__date__ = '30.05.2015'
__version__ = '1.0'

# Standard library imports
from math import *
import numpy as np
# Local imports
from Exercise2_new.util import PID as PID
from Exercise2_new.util import Calculations as Calc


class BasicMovement:
    """ Class Description:
    Contains basic methods for robot movement.
    Call __init__(robot_ref) first,
    then access methods.
    """

    def __init__(self, my_robot):
        """ Parameter initialization for necessary PID controllers
        :param my_robot: robot reference
        :return: -
        """
        # Robot reference
        self.robot = my_robot
        # PID parameters for follow_line()
        _k_p_fl = 0.6
        _k_i_fl = 0.001
        _k_d_fl = 0.8
        # PID parameters for rotate_to_target_point()
        _k_p_rot = 1.5
        _k_i_rot = 0.01
        _k_d_rot = 0.2
        # Time-step dt
        _dt = self.robot.getTimeStep()
        # Initial errors
        self.e_dist_old = 0
        self.int_e_dist_dt = 0

        # PID controllers
        self.pid_fl = PID.PID(_k_p_fl, _k_i_fl, _k_d_fl, _dt)
        self.pid_rot = PID.PID(_k_p_rot, _k_i_rot, _k_d_rot, _dt)

    def get_robot_pos(self):
        """ Return current robot position (x,y,theta)
        :return: robot position (x,y,theta)
        """
        return self.robot.getTrueRobotPose()

    def follow_line(self, p1, p2, v):
        """ Let the robot follow a line defined py points p1 & p2.
        Process is controlled by a PID controller (params in __init__ of BasicMovement)
        :param p1: point 1 of the line
        :param p2: point 2 of the line
        :param v: robot speed
        :return: speed, angular velocity ([v,omega])
        """

        [x, y, theta] = self.get_robot_pos()

        # Calculate distance e from point q to line g
        # line g = r1 + x * a
        # point q = [x, y]
        a = np.asarray(p2) - np.asarray(p1)
        r1 = np.asarray(p1)
        q = np.asarray([x, y])
        diff = q - r1
        e_dist = np.linalg.norm(np.cross(a, diff)) / np.linalg.norm(a)

        # Check if robot is on the left or on the right side of the line
        # Cross-product of the direction vectors of line g: (p2 - p1) and line between p1 and point q: (q - p1)
        # Right-hand rule determines the orientation of the resulting imaginary z-axis:
        # Positive: robot is on the right side of the line
        # Negative: robot is on the left side of the line
        z = a[1] * (x - p1[0]) - a[0] * (y - p1[1])

        e_dist *= -np.sign(z)

        omega = self.pid_fl.control(e_dist)

        return [v, omega]

    def rotate_to_target_point(self, point):
        """ Rotates robot to the specified point
        :param point: tuple(x,y)
        :return: speed, angular velocity ([v,omega])
        """

        # target direction
        [x, y, theta] = self.get_robot_pos()
        theta_target = atan2(point[1] - y, point[0] - x)

        omega = -self.pid_rot.control(Calc.diff(theta, theta_target))
        v = 0
        return [v, omega]
