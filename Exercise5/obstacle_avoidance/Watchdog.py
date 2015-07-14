# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
from Exercise3.util import Calculations as Calc

#TODO Watchdog sucht sich maxima täler ähnlich polarhistogramm

class Watchdog:

    def __init__(self, robot_location):
        # the map to avoid obstacles
        #self.occupancy_grid = occupancy_grid
        # information about the robot
        self.robot_location = robot_location
        # safety distance is simply added to robots radius to clear measurement faults
        self.safety_distance = 0.05
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
        self.threshold = 1.0
        # old threshold
        self.threshold_old = 1.0

    def apply_watchdog(self, movement_commands, target_direction = 0.0):
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
        # also check it on the middle position
        middle_position = [dx/2.0, dy/2.0]

        tolerance = self.robot_radius + self.safety_distance

        # storage if emergency was detected
        emergency = False

        # found point where the robot will crash into
        impact_point = None

        # check if robot crashes into one of the points
        for point in obstacle_points:
            # when true then robot will stall
            if Calc.point_in_tol(point, new_robot_position, tolerance) or Calc.point_in_tol(point, middle_position, tolerance):
                impact_point = point
                # set velocity to zero
                v = 0

                # check if robot is alreay in an emergency rotation just keep on rotating
                if self.emergency_rotation != 0:
                    omega = self.emergency_rotation
                    return [v, omega]


        # when an impact point was found
        if impact_point is not None:

            # function for rotation selection
            omega = self.rotate_to_next_free_edge()
            #omega = self.rotate_towards_target_direction(target_direction)

            # mark that there was an emergency rotation in the last time step
            self.emergency_rotation = omega
        else:
            self.emergency_rotation = 0

        return [v, omega]

    def move_robot(self, robot, movement_commands):
        # watchdog function to move robot
        [v, omega] = self.apply_watchdog(movement_commands)
        stalled = not robot.move([v, omega])

        # if robot is stalled rotate by ca 180 degrees
        if stalled:
            n = int(1/self.dT)
            for i in xrange(n):
                robot.move([0, self.omega_max])
            #robot.move([-self.v_max, omega])

        return [v, omega]


    def rotate_towards_target_direction(self, target_angle):
        """
        robot rotates to the edge whcih is the closest one to target direction
        :param target_angle:
        :return:
        """
        sensor_data = self.robot_location.get_sensor_data_above_threshold(self.threshold)

        # find closest angle with no obstacle in list
        min_angle_diff = 2*pi
        for [distance, angle] in sensor_data:
            angle_diff = Calc.diff(angle, target_angle)
            if abs(angle_diff) < abs(min_angle_diff):
                min_angle_diff = angle_diff

        # rotate towards min angle
        if -min_angle_diff < 0:
            omega = -self.omega_max
        else:
            omega = self.omega_max

        return omega


    def rotate_to_next_free_edge(self):
        """
        robot will rotate towards the next sensed direction which distance lies above threshold
        :param sensor_data:
        :return: omega
        """
        sensor_data = self.robot_location.get_sensor_data_above_threshold(self.threshold_old)

        # find closest angle with no obstacle in list
        min_angle = 2*pi
        for [distance, angle] in sensor_data:
            if abs(angle) < abs(min_angle):
                min_angle = angle

        # rotate towards min angle
        if min_angle < 0:
            omega = -self.omega_max
        else:
            omega = self.omega_max

        return omega


