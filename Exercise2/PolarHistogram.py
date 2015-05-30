# -*- coding: utf-8 -*-
__author__ = 'Ecki'

import numpy as np
from math import *

class PolarHistogram:

    def __init__(self, robot, robot_navigation):
        self.robot = robot
        self.robotNav = robot_navigation
        self.threshold = 0.05
        self.angle_threshold = 30*pi/180
        self.histogram = None
        self.k_p_omega = 1.2

    def avoidObstacle(self, target_point):
        direction_angle = self.robotNav.getAngleFromRobotToPoint(target_point)
        self.histogram = self.generateHistFromSensors()
        minima = self.locateMinima(self.histogram)
        #closest_angle = self.robotNav.searchClosestAngle(direction_angle, self.robotNav.getColumnFromList(2, minima))
        closest_angle = self.computeClosestAngle(direction_angle, minima)

        # search for closest angle



        print "closest angle: ", closest_angle
        omega = self.k_p_omega * closest_angle

        #v = self.robot._maxSpeed * 1 * (1 - omega/self.robot._maxOmega)
        v=0.4
        return [v, omega]

    # searches for closest angle in minima array
    def computeClosestAngle(self, target_angle, minima):
        closest_angles = []
        diff_old = 2*pi
        for angles in minima:
            diff = abs(self.robotNav.diff(angles[2], target_angle))
            if diff_old > diff:
                diff_old = diff
                closest_angles = angles

        if self.robotNav.AngleInRange(
                closest_angles[0], closest_angles[1], target_angle, True, self.angle_threshold):
            closest_angle = target_angle
        else:
            closest_angle = closest_angles[2]

        return closest_angle


    def generateHistFromSensors(self):
        """
        generates a dicrete histogram from robots sensor data
        :return:
        the histogram
        """
        sensorData = np.column_stack((self.robot._sensorDirections, self.robot.sense()))
        #print(sensorData)
        for i in range(len(sensorData)):
            if sensorData[i][1] == None:
                sensorData[i][1] = 0.0
            else:
                sensorData[i][1] = self.robot.getMaxSenseValue() - sensorData[i][1]
                if sensorData[i][1] < 0:
                    sensorData[i][1] = 0.0
        return sensorData

    # generates the 2D-List mininma
    # each minimum is described in one line with the colums: start, end, mid angle
    def locateMinima(self, histrogram):
        minima = []

        # recognizes if loop runs inside a minumum
        inside_minimum = False

        for i in range(len(histrogram)):

            # last element reached
            if i == (len(histrogram) - 1):

                # minimum doesn't end at last entry
                if histrogram[i][1] < self.threshold:

                    # minimum goes on at first entry
                    if histrogram[0][1]< self.threshold:
                        # if no minumum was found (no obstacle in sight)
                        if len(minima) == 0:
                            return [0.0, 0.0, 0.0]

                        # combine with first found minimum by start angle
                        minima[0][0] = start_angle
                    # minimum doesn't go on
                    else:

                        # check if minimum just has started
                        if not inside_minimum:
                            start_angle = histrogram[i][0]

                        end_angle = histrogram[i][0]
                        # save last minumum
                        minima.append([start_angle, end_angle, 0])


            # minimum starts
            elif histrogram[i][1] < self.threshold and not inside_minimum:
                inside_minimum = True
                # save starting angle of minimum
                start_angle = histrogram[i][0]


            # minmum ends
            elif histrogram[i][1] > self.threshold and inside_minimum:
                inside_minimum = False
                # save end angle of minimum
                end_angle = histrogram[i-1][0]

                minima.append([start_angle, end_angle, 0])

        # calculate mid angles
        for i in range(len(minima)):
            minima[i][2] = self.robotNav.getMedialAngle(minima[i][0], minima[i][1])

        return minima




