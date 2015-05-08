# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *
from HTWG_Robot_Simulator_V1 import Robot as robot
import numpy as np
import random

class Braitenberg:
    def __init__(self, my_robot):
        self.robot = my_robot
        self.maxSpeed = self.robot._maxSpeed
        self.maxOmega = self.robot._maxOmega
        self.maxSenseValue = self.robot._maxSenseValue


    def beScary(self):
        # from Robot2:

        # Get sensor distances and corresponding sensor directions
        allSensorDist = self.robot.sense()
        sensorDist = np.asarray([allSensorDist[6], allSensorDist[12]])
        sensorDirections = self.robot.getSensorDirections()

        for index, d in enumerate(sensorDist):
            if d is not None:
                # Scale distance to [0...1] anti-proportional
                d = 1.0 - d/self.maxSenseValue
            else:
                # If d is None, set it to zero
                d = 0
            sensorDist[index] = d

        # Set speed
        v = 2*self.maxSpeed - sensorDist[0] - sensorDist[1]

        # If there is no obstacle detected (sensorDist are both 0) let the robot turn around occasionally
        if not np.any(sensorDist):
            numberRandom = random.random()
            if numberRandom > 0.8:
                # turn left
                sensorDist[1] = 1.0
            elif numberRandom < 0.2:
                # turn right
                sensorDist[0] = 1.0

        # Set omega
        omega = self.maxOmega * (sensorDist[0] - sensorDist[1])


        return [v, omega]