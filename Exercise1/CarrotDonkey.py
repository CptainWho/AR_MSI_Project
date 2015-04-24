# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
from HTWG_Robot_Simulator_V1 import Robot as robot

class CarrotDonkey:
    def __init__(self, my_robot):
        self.e_omega_old = 0
        self.e_dist_old = 0
        self.int_e_omega_dt = 0
        self.int_e_dist_dt = 0
        self.robot = my_robot
        self.dt = self.robot.getTimeStep()
        # PID Regler für Omega
        self.k_p_omega = 0.2
        self.k_i_omega = 0
        self.k_d_omega = 0.2
        # PID Regler für v
        self.k_p_v = 0.2
        self.k_i_v = 0
        self.k_d_v = 0.3
        # distance to keep from dot
        self.space = 1
        # offset on v if its zero
        self.v_off = 0.0

    # calculate the angle difference from theta to theta_target
    # positive is defined as counterclockwise
    def diff(self, theta, theta_target):
        return (theta_target - theta + pi) % (2 * pi) - pi


    # returns [v, omega]. With v and omega as move input for the robot to follow a point p (carrot)
    def followCarrot(self, p):

        [x, y, theta] = self.robot.getTrueRobotPose()
        del_x = p[0] - x
        del_y = p[1] - y

        # calculate theta error
        theta_target = atan2(del_y, del_x)
        e_theta = -self.diff(theta, theta_target)
        # build derivative
        de_omega_dt = (e_theta - self.e_omega_old) / self.dt
        # build integral
        self.int_e_omega_dt += (e_theta + self.e_omega_old) * self.dt / 2
        # apply closed-loop
        omega = - self.k_p_omega * e_theta - self.k_d_omega * de_omega_dt - self.k_i_omega * self.int_e_omega_dt

        # calculate distance error
        dist = sqrt(del_x**2 + del_y**2)
        if dist < self.space:
            e_dist = 0
        else:
            e_dist = self.space - dist
        # build derivative
        de_dist_dt = (e_dist - self.e_dist_old) / self.dt
        # build integral
        self.int_e_dist_dt += (e_dist + self.e_dist_old) * self.dt / 2
        # apply closed-loop
        v = - self.k_p_v * e_dist - self.k_d_v * de_dist_dt - self.k_i_v * self.int_e_dist_dt
        # add offset
        v += self.v_off

        return [v, omega]
