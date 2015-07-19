# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
import numpy as np
from Exercise5.util import PID
from Exercise5.util import Calculations as Calc
from Exercise5.util import Polyline

class CarrotDonkey:
    def __init__(self, my_robot, my_world, robot_loc, move_backwards=True):
        self.robot = my_robot
        self.world = my_world
        self.dt = self.robot.getTimeStep()
        self.robot_loc = robot_loc
        # PID Regler für Omega
        self.k_p_omega = 2.5
        self.k_i_omega = 0.0
        self.k_d_omega = 0.5
        self.pid_omega = PID.PID(self.k_p_omega, self.k_i_omega, self.k_d_omega)
        # PID Regler für v
        self.k_p_v = 3.8
        self.k_i_v = 0.002
        self.k_d_v = 0.1
        #self.k_d_v = 0.1
        self.pid_v = PID.PID(self.k_p_v, self.k_i_v, self.k_d_v)
        # distance to keep from dot
        self.space = 0.3  # 0.3
        # distance where carrot waits for robot
        self.max_space = self.space*3.0
        # offset on v if its zero
        self.v_off = 0.0
        # define minimum tolerance to carrot
        self.tolerance = 0.01
        # the carrot
        self.carrot = Carrot(my_robot, my_world, robot_loc, space=self.space*1.8)
        # give the carrot the permission to move backwards, when robot does
        self.move_backwards = move_backwards

    # Added 16.07.2015 (Phil)
    def get_carrot_position(self):
        return self.carrot.get_pos()

    def set_polyline(self, polyline, v):
        self.carrot.set_polyline(polyline, v)

    # calculate the angle difference from theta to theta_target
    # positive is defined as counterclockwise
    def diff(self, theta, theta_target):
        return (theta_target - theta + pi) % (2 * pi) - pi

    def set_space(self, space):
        """
        sets the space the robot keeps between himself and the carrot
        :param space: space in meters
        """
        self.space = space

    # returns [v, omega] for one time step. With v and omega as move input for the robot to follow a point (the carrot)
    def follow_carrot(self):

        carrot = self.carrot.get_pos()

        # when robot has reached carrot, don't anything
        if self.robot_loc.robot_inside_point_tolerance(carrot, self.tolerance):
            return [0, 0]

        [x, y] = self.robot_loc.get_robot_point()
        del_x = carrot[0] - x
        del_y = carrot[1] - y

        # get omega
        [v, omega] = self.rotate_to_carrot()

        # when carrot has reached end of polyline, drive to carrot
        if self.carrot.end_point_reached():
            space = 0
            #return [1, omega] #uncomment to drive with max speed to parking carrot
        else:
            space = self.space

        #### PID for v
        # calculate distance error
        dist = sqrt(del_x**2 + del_y**2)
        # if too close to carrot wait
        if dist < space:
            e_dist = 0
        else:
            e_dist = space - dist
        # apply pid control
        v = self.pid_v.control(e_dist)
        # add offset
        v += self.v_off

        return [v, omega]

    def rotate_to_carrot(self):
        """
        robot rotates until it looks to the carrot
        :return:
        """
        carrot = self.carrot.get_pos()
        return self.rotate_to_point(carrot)

    def rotate_to_point(self, point):
        """
        robot rotates until it looks to the given point
        :return:
        """
        [x, y, theta] = self.robot_loc.get_robot_position()
        del_x = point[0] - x
        del_y = point[1] - y

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
        carrot_pos = self.carrot.get_pos()
        robot_pos = self.robot_loc.get_robot_point()
        distance = Calc.get_dist_from_point_to_point(carrot_pos, robot_pos)
        if distance > self.max_space:
            return True
        else:
            return False

    def get_next_point(self):
        return self.carrot.p_next()

    def end_point_reached(self, tol):
        end_point = self.carrot.p_end()
        robot_point = self.robot_loc.get_robot_point()
        return Calc.point_in_tol(robot_point, end_point, tol)

    def robot_closer_than_carrot(self, point, carrot_point):
        """
        checks if robot is closer to a certain point than the carrot
        :param point:
        :return: True/False
        """
        robot_dist = Calc.get_dist_from_point_to_point(point, self.robot_loc.get_robot_point())
        carrot_dist = Calc.get_dist_from_point_to_point(point, carrot_point)
        if robot_dist < carrot_dist:
            return True
        else:
            return False

    def place_carrot_above_robot(self):
        self.carrot.place_carrot_above_robot(self.move_backwards)
        self.reset_pid_controllers()

    def reset_pid_controllers(self):
        self.pid_omega.clear_storage()
        self.pid_v.clear_storage()

    def next_movement_commands(self):

        robot_too_far_away = self.robot_too_far_away()

        carrot_point = self.carrot.get_pos()

        # when robot is closer to target point, than carrot itself, set carrot to projection of robot on the line
        if self.robot_closer_than_carrot(self.carrot.p_next(), carrot_point):
            self.carrot.place_carrot_above_robot(self.move_backwards)

        # # when robot comes too close to carrot
        # if self.robot_too_close_to_carrot():
        #     rel_point = Calc.get_orthogonal_point_on_line(self.carrot_pos, self.p_next, self.robot_loc.get_robot_point(), self.space*1.4)
        #     new_carrot_pos = [self.carrot_pos[0] + rel_point[0], self.carrot_pos[1] + rel_point[1]]
        #     self.set_carrot_position(new_carrot_pos)

        # if robot is far away from line, just place carrot below robot on the line
        # if not robot_close_to_line:
        #     self.set_carrot_position(Calc.get_orthogonal_point_on_line(self.p_last, self.p_next, self.robot_loc.get_robot_point(), offset=0))

        # carrot waits if robot is too far away
        if robot_too_far_away:
            movement_commands = self.follow_carrot()
            return movement_commands

        # move carrot as long as end of polyline is not reached
        if not self.carrot.end_point_reached():
            self.carrot.move_carrot_to_point(self.carrot.p_next(), self.carrot.v_carrot)
            # when next point reached increase index
            if self.carrot.get_pos() == self.carrot.p_next():
                self.carrot.update_polyline()

        movement_commands = self.follow_carrot()
        return movement_commands

class Carrot:

    def __init__(self, robot, world, robot_loc, space=0.5, draw_carrot=True):
        self.robot_loc = robot_loc
        self.world = world
        # Time difference
        self.dt = robot.getTimeStep()
        # carrot speed limit
        self.v_max = robot.getMaxSpeed()
        # define position of carrot
        self.carrot_pos = [10, 10]
        # the polyline to follow
        self.poly = Polyline.Polyline(None)
        # speed of carrot
        self.v_carrot = 0
        # carrot visible?
        self.draw_carrot = draw_carrot
        # distance the carrot keeps to robot
        self.space = space

    def set_carrot_position(self, point):
        """
        sets a point as carrot to specified coordiantes
        :param point:
        :return:
        """
        self.carrot_pos = point
        if self.draw_carrot:
            self.world.drawCircle(point, color="orange")

    def get_pos(self):
        """
        :return: the carrots current position
        """
        return self.carrot_pos

    def p_next(self):
        return self.poly.p_next()

    def p_last(self):
        return self.poly.p_last()

    def p_end(self):
        return self.poly.p_end()

    def last_point_reached(self):
        if self.carrot_pos == self.poly.p_last():
            return True
        else:
            return False

    def end_point_reached(self):
        if self.carrot_pos == self.poly.p_end():
            return True
        else:
            return False

    def set_polyline(self, polyline, v):
        """
        sets the polyline and the speed to follow it
        :param polyline:
        :param v:
        :return:
        """
        self.poly.set_polyline(polyline)
        self.v_carrot = Calc.limit_value(v, self.v_max)
        # place carrot at start point
        self.set_carrot_position(self.poly.p_last())

    def update_polyline(self):
        self.poly.update_points()

    def move_carrot_to_point(self, p, v):
        """
        moves carrot in a straight line to specified point with v
        :param p:
        :param v:
        :return:
        """
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
            del_x_new = p[0] - p_new[0]
            del_y_new = p[1] - p_new[1]
            d_new = sqrt(del_y_new**2 + del_x_new**2)

            # stop if point is reached
            if d_new < d_diff/2:
                self.set_carrot_position(p)
            # stop if distance gets bigger again
            elif d_new > d:
                self.set_carrot_position(p)
            # else renew position
            else:
                self.set_carrot_position(p_new)

    def place_carrot_above_robot(self, move_backwards=True):
        """
        always use when robot is moved without carrot donkey
        moves the carrot onto the polyline parallel to the robot
        :return:
        """

        new_carrot_pos = self.get_point_orthogonal_to_robot()
        carrot_dist = Calc.get_dist_from_point_to_point(self.poly.p_last(), new_carrot_pos)
        line_dist = Calc.get_dist_from_point_to_point(self.poly.p_last(), self.poly.p_next())

        # check if carrot would move backwards (comes closer to last point)
        if not move_backwards:
            carrot_dist_old = Calc.get_dist_from_point_to_point(self.poly.p_last(), self.carrot_pos)
            if carrot_dist_old > carrot_dist:
                # if so don't move carrot
                new_carrot_pos = self.carrot_pos

        # if carrot would get past next point update the polyline and check again
        while carrot_dist > line_dist or self.poly.p_next() == new_carrot_pos:
            # update to next point
            self.poly.update_points()
            new_carrot_pos = self.get_point_orthogonal_to_robot()
            carrot_dist = Calc.get_dist_from_point_to_point(self.poly.p_last(), new_carrot_pos)
            line_dist = Calc.get_dist_from_point_to_point(self.poly.p_last(), self.poly.p_next())

        # if carrot is ahead of last point, move it to last point
        carrot_dist_end = Calc.get_dist_from_point_to_point(new_carrot_pos, self.poly.p_next())
        if carrot_dist_end > line_dist:
            new_carrot_pos = self.poly.p_last()

        # set the carrot position
        self.set_carrot_position(new_carrot_pos)

    def get_point_orthogonal_to_robot(self):
        p_last = self.poly.p_last()
        p_next = self.poly.p_next()
        p_robot = self.robot_loc.get_robot_point()
        rel_point = Calc.get_orthogonal_point_on_line(p_last, p_next, p_robot, offset=self.space)
        new_point = [p_last[0] + rel_point[0], p_last[1] + rel_point[1]]
        return new_point

    def adjust_carrot_pos(self, suggested_pos):
        # check if carrot would be at or beyond point
        line_dist = self.poly.get_dist_between_points()
        carrot_dist = Calc.get_dist_from_point_to_point(self.poly.p_last(), suggested_pos)
        if carrot_dist > line_dist or self.poly.p_next() == suggested_pos:
            # update to next point
            self.poly.update_points()
            # set carrot to start
            self.set_carrot_position(self.poly.p_last())

    # OBSOLETE
    # def carrot_move_with_robot(self):
    #     """
    #     always use when robot is moved without carrot donkey
    #     moves the carrot onto the polyline parallel to the robot
    #     :return:
    #     """
    #
    #     new_carrot_pos = self.get_point_orthogonal_to_robot()
    #     carrot_dist = Calc.get_dist_from_point_to_point(self.poly.p_last(), new_carrot_pos)
    #     line_dist = Calc.get_dist_from_point_to_point(self.poly.p_last(), self.poly.p_next())
    #
    #     # if carrot would get past next point update the polyline and check again
    #     while carrot_dist > line_dist or self.poly.p_next() == new_carrot_pos:
    #         # update to next point
    #         self.poly.update_points()
    #         new_carrot_pos = self.get_point_orthogonal_to_robot()
    #         carrot_dist = Calc.get_dist_from_point_to_point(self.poly.p_last(), new_carrot_pos)
    #         line_dist = Calc.get_dist_from_point_to_point(self.poly.p_last(), self.poly.p_next())
    #
    #     # if carrot is ahead of last point, move it to last point
    #     carrot_dist_end = Calc.get_dist_from_point_to_point(new_carrot_pos, self.poly.p_next())
    #     if carrot_dist_end > line_dist:
    #         new_carrot_pos = self.poly.p_last()
    #
    #     # set the carrot position
    #     self.set_carrot_position(new_carrot_pos)


