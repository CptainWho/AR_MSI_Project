# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
import numpy as np

class CarrotDonkey:
    def __init__(self, my_robot, my_world):
        self.e_omega_old = 0
        self.e_dist_old = 0
        self.int_e_omega_dt = 0
        self.int_e_dist_dt = 0
        self.robot = my_robot
        self.world = my_world
        self.dt = self.robot.getTimeStep()
        # PID Regler für Omega
        self.k_p_omega = 0.2
        self.k_i_omega = 0.00
        self.k_d_omega = 0.2
        # PID Regler für v
        self.k_p_v = 0.2
        self.k_i_v = 0.00
        self.k_d_v = 0.3
        # distance to keep from dot
        self.space = 0.1
        # offset on v if its zero
        self.v_off = 0.0
        # define position of carrot
        self.carrot_pos = [10, 10]


    # calculate the angle difference from theta to theta_target
    # positive is defined as counterclockwise
    def diff(self, theta, theta_target):
        return (theta_target - theta + pi) % (2 * pi) - pi


    # sets a point as carrot to specified coordiantes
    def setCarrotPosition(self, p):
        self.carrot_pos = p
        self.world.drawCircle(p)

    def set_space(self, space):
        """
        sets the space the robot keeps between himself and the carrot
        :param space: space in meters
        """
        self.space = space


    # returns a list of coordinates so that a point can travel on the line with v
    # (last point not included)
    def getLinePoints(self, p_start, p_end, v):

        # total distances
        del_x = p_end[0] - p_start[0]
        del_y = p_end[1] - p_start[1]
        d = sqrt(del_y**2 + del_x**2)
        steps = int(np.round(d / (v * self.dt), decimals=0))

        diff_x = del_x / steps
        diff_y = del_y / steps

        linePoints = []
        x_val = p_start[0]
        y_val = p_start[1]

        linePoints.append([x_val, y_val])
        for i in range(steps - 1):
            x_val += diff_x
            y_val += diff_y
            linePoints.append([x_val, y_val])

        return linePoints

    # moves carrot in a straight line to specified point with v
    def moveCarrotToPoint(self, p, v):
        # driving distance
        d_diff = v * self.dt
        # distance to point
        del_x = p[0] - self.carrot_pos[0]
        del_y = p[1] - self.carrot_pos[1]
        d = sqrt(del_y**2 + del_x**2)
        # new point
        p_new = [self.carrot_pos[0] + del_x * d_diff / d, self.carrot_pos[1] + del_y * d_diff / d]
        # new distance
        del_x = p[0] - p_new[0]
        del_y = p[1] - p_new[1]
        d_new = sqrt(del_y**2 + del_x**2)

        # stop if point is within tolerance
        if d_new < d_diff/2:
            self.setCarrotPosition(p)
        # stop if distance gets bigger again
        elif d_new > d:
            self.setCarrotPosition(p)
        # else renew position
        else:
            self.setCarrotPosition(p_new)


    # returns [v, omega] for one time step. With v and omega as move input for the robot to follow a point (the carrot)
    def followCarrot(self):

        carrot = self.carrot_pos

        [x, y, theta] = self.robot.getTrueRobotPose()
        del_x = carrot[0] - x
        del_y = carrot[1] - y

        # get omega
        [v, omega] = self.rotateToCarrot()

        #### PID for v
        # calculate distance error
        dist = sqrt(del_x**2 + del_y**2)
        if dist < self.space:
            e_dist = 0
        else:
            e_dist = self.space - dist
        # build derivative
        de_dist_dt = (e_dist - self.e_dist_old) / self.dt
        # build integral
        self.int_e_dist_dt -= (e_dist + self.e_dist_old) * self.dt / 2
        # apply closed-loop
        v = - self.k_p_v * e_dist - self.k_d_v * de_dist_dt - self.k_i_v * self.int_e_dist_dt
        # add offset
        v += self.v_off

        return [v, omega]

    def rotateToCarrot(self):
        carrot = self.carrot_pos
        [x, y, theta] = self.robot.getTrueRobotPose()
        del_x = carrot[0] - x
        del_y = carrot[1] - y

        #### PID for omega
        # calculate theta error
        theta_target = atan2(del_y, del_x)
        e_theta = -self.diff(theta, theta_target)
        # build derivative
        de_omega_dt = (e_theta - self.e_omega_old) / self.dt
        # build integral
        self.int_e_omega_dt += (e_theta + self.e_omega_old) * self.dt / 2
        # apply closed-loop
        omega = - self.k_p_omega * e_theta - self.k_d_omega * de_omega_dt - self.k_i_omega * self.int_e_omega_dt
        # return omega
        v = 0
        return [v, omega]