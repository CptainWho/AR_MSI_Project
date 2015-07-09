# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
import numpy as np
from Exercise5.util import RobotLocation
from Exercise5.util import PID
from Exercise5.util import Calculations as Calc

class CarrotDonkey:
    def __init__(self, my_robot, my_world):
        self.robot = my_robot
        self.world = my_world
        self.dt = self.robot.getTimeStep()
        self.robot_loc = RobotLocation.RobotLocation(my_robot)
        # PID Regler für Omega
        self.k_p_omega = 2.5
        self.k_i_omega = 0.00
        self.k_d_omega = 0.4
        self.pid_omega = PID.PID(self.k_p_omega, self.k_i_omega, self.k_d_omega)
        # PID Regler für v
        self.k_p_v = 1.2
        self.k_i_v = 0.00
        self.k_d_v = 0.1
        self.pid_v = PID.PID(self.k_p_v, self.k_i_v, self.k_d_v)
        # distance to keep from dot
        self.space = 0.3
        # distance where carrot waits for robot
        self.max_space = 1.5
        # offset on v if its zero
        self.v_off = 0.0
        # define position of carrot
        self.carrot_pos = [10, 10]
        self.carrot_pos_old = self.carrot_pos
        # define minimum tolerance to carrot
        self.tolerance = 0.01
        # the polyline to follow
        self._polyline = None
        # the speed of the carrot
        self.v_carrot = 0
        # index from the polyline
        self.line_index = 0
        # next point
        self.p_next = None
        # last point
        self.p_last = None

    def set_polyline(self, polyline, v):
        # sets the polyline and the speed to follow it
        self._polyline = polyline
        self.v_carrot = Calc.limit_value(v, self.robot.getMaxSpeed())
        # set index from polyline
        if len(polyline) > 1:
            self.line_index = 1
            self.p_next = polyline[1]
        else:
            self.line_index = 0
        # place carrot at start point
        self.p_last = polyline[0]
        self.set_carrot_position(self.p_last)

    # calculate the angle difference from theta to theta_target
    # positive is defined as counterclockwise
    def diff(self, theta, theta_target):
        return (theta_target - theta + pi) % (2 * pi) - pi

    def get_next_point(self):
        return self.p_next

    def get_last_point(self):
        return self.p_last

    # sets a point as carrot to specified coordiantes
    def set_carrot_position(self, p):
        self.carrot_pos = p
        self.world.drawCircle(p)

    def get_carrot_position(self):
        """
        :return: the carrots current position
        """
        return self.carrot_pos

    def set_space(self, space):
        """
        sets the space the robot keeps between himself and the carrot
        :param space: space in meters
        """
        self.space = space

    # moves carrot in a straight line to specified point with v
    def move_carrot_to_point(self, p, v):
        # driving distance
        d_diff = v * self.dt
        # distance to point
        del_x = p[0] - self.carrot_pos[0]
        del_y = p[1] - self.carrot_pos[1]
        d = sqrt(del_y**2 + del_x**2)
        # if distance is not zero
        if not d == 0:
            # new point
            p_new = [self.carrot_pos[0] + del_x * d_diff / d, self.carrot_pos[1] + del_y * d_diff / d]
            # new distance
            del_x = p[0] - p_new[0]
            del_y = p[1] - p_new[1]
            d_new = sqrt(del_y**2 + del_x**2)

            # stop if point is within tolerance
            if d_new < d_diff/2:
                self.set_carrot_position(p)
            # stop if distance gets bigger again
            elif d_new > d:
                self.set_carrot_position(p)
            # else renew position
            else:
                self.set_carrot_position(p_new)

    def __get_internal_space(self):
        """
        check if carrot pos is changing.
        if not carrot is stationary and robot dirves directly to it
        :return:
        """
        if self.carrot_pos == self.carrot_pos_old:
            return 0
        else:
            return self.space
        #return self.space

    # returns [v, omega] for one time step. With v and omega as move input for the robot to follow a point (the carrot)
    def follow_carrot(self):

        # when robot has reached carrot, don't anything
        if self.robot_loc.robot_inside_point_tolerance(self.carrot_pos, self.tolerance):
            return [0, 0]

        carrot = self.carrot_pos

        [x, y, theta] = self.robot.getTrueRobotPose()
        del_x = carrot[0] - x
        del_y = carrot[1] - y

        # get omega
        [v, omega] = self.rotate_to_carrot()

        #### PID for v
        # calculate distance error
        dist = sqrt(del_x**2 + del_y**2)
        if dist < self.__get_internal_space():
            e_dist = 0
        else:
            e_dist = self.__get_internal_space() - dist
        # apply pid control
        v = self.pid_v.control(e_dist)
        # add offset
        v += self.v_off

        # save old carrot position
        self.carrot_pos_old = self.carrot_pos

        return [v, omega]

    def rotate_to_carrot(self):
        carrot = self.carrot_pos
        [x, y, theta] = self.robot.getTrueRobotPose()
        del_x = carrot[0] - x
        del_y = carrot[1] - y

        #### PID for omega
        # calculate theta error
        theta_target = atan2(del_y, del_x)
        e_theta = -self.diff(theta, theta_target)
        # apply pid control
        omega = self.pid_omega.control(e_theta)
        # return omega
        v = 0
        return [v, omega]

    def robot_too_far_away(self):
        """
        checks if robot is too far away
        :return: True/False
        """
        carrot_pos = self.carrot_pos
        robot_pos = self.robot_loc.get_robot_point()
        distance = Calc.get_dist_from_point_to_point(carrot_pos, robot_pos)
        if distance > self.max_space:
            return True
        else:
            return False

    def robot_close_to_line(self):
        """
        checks if robot is close to to the given line
        :return:
        """
        dist = Calc.get_signed_distance_from_line_to_point(self.p_last, self.p_next, self.robot_loc.get_robot_point())
        if abs(dist) < self.max_space:
            return True
        else:
            return False

    def robot_closer_than_carrot(self, point):
        """
        checks if robot is closer to a certain point than the carrot
        :param point:
        :return: True/False
        """
        robot_dist = Calc.get_dist_from_point_to_point(point, self.robot_loc.get_robot_point())
        carrot_dist = Calc.get_dist_from_point_to_point(point, self.get_carrot_position())
        if robot_dist < carrot_dist:
            return True
        else:
            return False

    def next_movement_commands(self):

        robot_too_far_away = self.robot_too_far_away()

        # when robot is closer to target point, than carrot itself, set carrot to projection of robot on the line
        if self.robot_closer_than_carrot(self.p_next):
            # when robot is still close to carrot, add offset
            if self.robot_close_to_line():
                offset = self.space*2
            else:
                offset = 0
            # Calculate new point for carrot
            rel_point = Calc.get_orthogonal_point_on_line(self.p_last, self.p_next, self.robot_loc.get_robot_point(), offset)
            point = [self.p_last[0] + rel_point[0], self.p_last[1] + rel_point[1]]
            self.set_carrot_position(point)

        # carrot waits if robot is too far away
        if robot_too_far_away:
            movement_commands = self.follow_carrot()
            return movement_commands

        # move carrot as long as end of polyline is not reached
        if self.line_index < len(self._polyline):
            self.p_next = self._polyline[self.line_index]
            self.move_carrot_to_point(self.p_next, self.v_carrot)
            # when next point reached increase index
            if self.carrot_pos == self.p_next:
                self.line_index += 1
                self.p_last = self.p_next

        movement_commands = self.follow_carrot()
        return movement_commands

