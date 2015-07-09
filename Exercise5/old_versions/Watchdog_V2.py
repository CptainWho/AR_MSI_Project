# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
from Exercise3.util import Calculations as Calc

class Watchdog:

    def __init__(self, robot_location):
        # the map to avoid obstacles
        #self.occupancy_grid = occupancy_grid
        # information about the robot
        self.robot_location = robot_location
        # safety distance is simply added to robots radius to clear measurement faults
        self.safety_distance = 0.1
        # the robots radius
        self.robot_radius = self.robot_location.get_robot_radius()
        # the robots maximum speed
        self.v_max = self.robot_location.get_max_robot_speed()
        # the robots maximum omega
        self.omega_max = self.robot_location.get_max_robot_omega()
        # time delta
        self.dT = 0.1
        # indicates if robot is rotating to avoid an obstacle
        # if so this variable has a rotation value, else it is zero
        # if not zero robot should keep this rotation
        self.emergency_rotation = 0
        # threshold value for emergency rotation
        self.threshold = 0.5

    def apply_watchdog(self, movement_commands):
        [v, omega] = movement_commands

        obstacle_points = self.robot_location.get_relative_obstacle_points()

        # translational and rotational speed is limited:
        if omega > self.omega_max:
            omega = self.omega_max
        if omega < -self.omega_max:
            omega = -self.omega_max
        if v > self.v_max:
            v = self.v_max
        if v < -self.v_max:
            v = -self.v_max

        # Calculate position if robot applys move command
        # this position is relative to robots actual position
        distance = v * self.dT
        theta = omega * self.dT
        dx = distance * cos(theta)
        dy = distance * sin(theta)
        new_robot_position = [dx, dy]

        tolerance = self.robot_radius + self.safety_distance

        # storage if emergency was detected
        emergency = False

        # check if robot crashes into one of the points
        for point in obstacle_points:
            # when true then robot will stall
            if Calc.point_in_tol(point, new_robot_position, tolerance):
                emergency = True
                # set velocity to zero
                v = 0

                # first check if robot is alreay in an emergency rotation just keep on rotating
                if self.emergency_rotation != 0:
                    omega = self.emergency_rotation
                    return [v, omega]

                # when robot drives straight ahead always rotate positive
                if omega == 0.0:
                    omega = self.omega_max
                # otherwise turn away from point
                else:
                    if 0 > self.robot_location.get_angle_from_robot_to_point(point):
                        omega = self.omega_max
                    else:
                        omega = -self.omega_max

                # mark that there was an emergency rotation in the last time step
                self.emergency_rotation = omega

        # reset emergency rotation
        if emergency == False:
            self.emergency_rotation = 0

        return [v, omega]





