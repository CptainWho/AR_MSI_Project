# -*- coding: utf-8 -*-
""" Module Description:
Implements Braitenberg behaviour for obstacle avoidance -> scary robot
"""

__author1__ = 'Ecki'
__author2__ = 'Philipp Lohrer'
__date__ = '30.05.2015'
__version__ = '1.0'

# Standard library imports
import numpy as np
from math import pi
# Local imports


class Braitenberg:
    """
    Implemented Braitenberg behaviour: be_scary()
    Call __init__() before accessing any method
    """

    def __init__(self, my_robot):
        """ Initialization
        :param my_robot: robot reference
        :return: -
        """

        self.robot = my_robot
        self.max_speed = self.robot.getMaxSpeed()
        self.max_omega = self.robot.getMaxOmega()
        self.max_sense_value = self.robot.getMaxSenseValue()
        self.sensor_left = 13
        self.sensor_right = 5

        # Info: print angles in degrees of left & right sensor
        sensor_directions = np.asarray(self.robot.getSensorDirections())
        sensor_directions *= 180 / pi
        print 'Sensor angles:\n\t left: %0.2f°, right: %0.2f°' % \
              (sensor_directions[self.sensor_left], sensor_directions[self.sensor_right])

    def be_scary(self):
        """ Simulates a scared robot behaviour -> robot tries to avoid detected obstacles
        :return: speed, angular velocity ([v,omega])
        """

        # Get sensor distances and corresponding sensor directions
        all_sensor_dist = self.robot.sense()
        sensor_dist = np.asarray([all_sensor_dist[self.sensor_right], all_sensor_dist[self.sensor_left]])

        for index, d in enumerate(sensor_dist):
            if d is not None:
                # Scale distance to [0...1] anti-proportional
                d = 1.0 - d/self.max_sense_value
            else:
                # If d is None, set it to zero
                d = 0
            sensor_dist[index] = d

        # Set speed
        v = 2.0 * self.max_speed - sensor_dist[0] - sensor_dist[1]

        # If there is no obstacle detected (sensorDist are both 0) let the robot turn around occasionally
        if not np.any(sensor_dist):
            random_number = np.random.random()
            if random_number > 0.8:
                # turn left
                sensor_dist[1] = 1.0
            elif random_number < 0.2:
                # turn right
                sensor_dist[0] = 1.0

        # Set omega
        omega = self.max_omega * (sensor_dist[0] - sensor_dist[1])

        return [v, omega]