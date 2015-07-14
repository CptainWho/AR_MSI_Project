# -*- coding: utf-8 -*-
""" Module RobotLocation
"""

__project__ = 'Exercise 5'
__module__  = 'RobotLocation'
__author1__  = 'Daniel Eckstein'
__author2__  = 'Philipp Lohrer'
__date__    = '14.07.2015'

__version__ = '1.0'

# Changelog:
# 14.07.2015:   added localization via particle filter MCL

from math import *
from Exercise5.util import Calculations as Calc
from Exercise4.localization import ParticleCloud, MCL

class RobotLocation:

    def __init__(self, robot, world, landmark_positions):
        """
        :param robot:
        :param world:
        :param landmark_positions:
        :return:
        """

        self.robot = robot
        self.world = world
        # Landmark positions for MCL
        self.landmark_positions = landmark_positions

        # Set up particle cloud and add particles
        particle_cloud = ParticleCloud.ParticleCloud(self.world, self.robot, draw='estimation')
        particle_cloud.create_particles(500, position=self.get_robot_position(est=False))

        # Set up MCL localization
        self.mcl = MCL.MCL(particle_cloud, robot_loc=self, draw=False)

        self.angle_tol = 0*pi/180

        # Update estimated position of the robot
        self.robot_position_est = None
        self.update_robot_position_est([0, 0])

    def update_robot_position_est(self, movement):
        """ Update estimated robot position with given movement
        :param movement:    [v, omega]
        :return:            -
        """

        number_dist_angles = self.robot.sense_landmarks()
        # Run MCL
        self.robot_position_est = self.mcl.mcl_landmark(movement, self.landmark_positions,
                                                        sensor_data=number_dist_angles, debug=False)

    def get_robot_position(self, est=True):
        """ Return estimated robot position if est id True (localization with MCL), otherwise return true robot position
        :param movement:    [v, omega], default None, used for MCL solely
        :return: [x, y, theta]
        """

        if est:
            return self.robot_position_est
        else:
            return self.robot.getTrueRobotPose()

    def get_robot(self):
        """
        returns the robot object for other functions
        :return:
        """
        return self.robot

    def get_time_step(self):
        """
        :return: time step
        """
        return self.robot.getTimeStep()

    def get_robot_point(self, est=True):
        """
        :return: robots x and y value in global coordinate system
        """
        [x, y, theta] = self.get_robot_position(est=est)
        return [x, y]

    def get_robot_angle(self, est=True):
        """
        :return: robots angle in global coordinate system
        """
        [x, y, theta] = self.get_robot_position(est=est)
        return theta

    def get_positive_robot_angle(self):
        """
        :return: robots positive angle in global coordinate system
        """
        theta = self.get_robot_angle() % (2 * pi)
        return theta

    def aiming_to_point(self, point, tolerance):
        """
        checks if robot is aming to a point
        :return: True/False
        """
        return Calc.angle_in_tol(self.get_robot_angle(),
                                 Calc.get_angle_from_point_to_point(self.get_robot_point(), point), tolerance)

    def robot_inside_tolerance(self, p_target, tolerance, robot_pos):
        """
        check whether robot in within tolerance of target point
        :param p_target: the target point [x, y]
        :param tolerance: tolaerance radius around the point
        :param robot_pos: [x, y, theta]
        :return: True or False
        """
        p = robot_pos[0:2]
        return Calc.point_in_tol(p, p_target, tolerance)

    def robot_inside_point_tolerance(self, p_target, tolerance):
        """
        check whether robot in within tolerance of target point
        :param p_target: the target point [x, y]
        :param tolerance: tolaerance radius around the point
        :param robot_pos: [x, y, theta]
        :return: True or False
        """
        p = self.get_robot_point()
        return Calc.point_in_tol(p, p_target, tolerance)

    def get_angle_from_robot_to_point(self, point):
        """
        returns the angle between robot position and a point
        :param point:
        :return:
        """
        theta = self.get_robot_angle()
        robot_point = self.get_robot_point()
        theta_target = Calc.get_angle_from_point_to_point(robot_point, point)
        diff = Calc.diff(theta, theta_target)
        return diff

    def get_diff_from_robot_to_point(self, point):
        """

        :param point:
        :return:
        """

    def get_robot_radius(self):
        """
        :return: the robots radius
        """
        diameter = self.robot.getSize()
        return diameter/2.0

    def get_max_robot_speed(self):
        return self.robot.getMaxSpeed()

    def get_max_robot_omega(self):
        return self.robot.getMaxOmega()

    def get_sensor_data_polar(self):
        angles = self.robot.getSensorDirections()
        distances = self.robot.sense()
        sensor_data = [distances, angles]
        return sensor_data

    def get_sensor_data_above_threshold(self, threshold):
        """
        returns robots polar sensor data
        but just the data where distance lies above specified threshold
        :return: list with [distances, angles]
        """
        new_data = []
        [distances, angles] = self.get_sensor_data_polar()

        for i in xrange(len(distances)):
            distance = distances[i]
            angle = angles[i]
            if distance > threshold or distance is None:
                new_data.append([distance, angle])

        return new_data

    def get_relative_obstacle_points(self):
        """
        returns alls points where the robot detects an obstacle
        the point position is seen relative from robot
        this means robot is always seen at point [0, 0] in coordinate system
        :return: a list of points
        """
        sensor_data = self.get_sensor_data_polar()
        [distances, angles] = sensor_data

        # list of of obstacle point
        obstacle_points = []

        # convert polar to cartesian
        for i in xrange(len(distances)):
            # sort out beams without obstacle
            if distances[i] is not None:
                rel_point = Calc.polar_2_cartesian(distances[i], angles[i])

                obstacle_points.append(rel_point)

        return obstacle_points
