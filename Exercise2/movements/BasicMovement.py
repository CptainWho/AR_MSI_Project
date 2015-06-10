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
from Exercise2.util import PID as PID
from Exercise2.util import Calculations as Calc
from Exercise2.util import RobotLocation


class BasicMovement:
    """ Class Description:
    Contains basic methods for robot movements.
    Call __init__(robot_ref) first,
    then access methods.
    """

    def __init__(self, robot):
        """ Parameter initialization for necessary PID controllers
        :param robot: robot reference
        :return: -
        """
        # Robot reference
        self.robot = robot
        # PID parameters for follow_line()
        _k_p_fl = 0.8
        _k_i_fl = 0.001
        _k_d_fl = 0.6
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

        # robot location
        self.RobLoc = RobotLocation.RobotLocation(self.robot)



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

        # check if robot rotates to far
        line_angle = Calc.get_positive_angle_of_line(p1, p2)
        robot_angle = self.RobLoc.get_positive_robot_angle()
        # if robot is on the left side
        if z < 0:
            start_angle = Calc.add_angles_positive(line_angle, -pi)
            end_angle = Calc.add_angles_positive(line_angle, -pi/2.0)
        # if robot is on the right side
        else:
            start_angle = Calc.add_angles_positive(line_angle, pi/2.0)
            end_angle = Calc.add_angles_positive(line_angle, pi)

        # when robot would turn away from line
        if Calc.angle_in_range(start_angle, end_angle, robot_angle):
                omega = -omega

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

    # TODO: Go to point implementieren!
    def goto_point(self, point, v=1.0):
        [x, y, theta] = self.get_robot_pos()
        theta_target = Calc.get_angle_from_robot_to_point([x, y, theta], point)
        omega = Calc.diff(theta, theta_target)

        return [v, omega]

