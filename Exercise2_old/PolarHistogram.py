# -*- coding: utf-8 -*-
__author__ = 'Ecki'

import numpy as np
from math import *
from Exercise2.util import Calculations as Calc

class PolarHistogram:

    def __init__(self, robot, robot_navigation):
        self.robot = robot
        self.robotNav = robot_navigation
        self.threshold = 0.05
        self.angle_threshold = 30*pi/180
        self.histogram = None
        self.k_p_omega = 0.8

    def avoidObstacle(self, target_point):
        direction_angle = self.robotNav.getAngleFromRobotToPoint(target_point)
        self.histogram = self.generateHistFromSensors()
        # minima = self.locateMinima(self.histogram)
        # #closest_angle = self.robotNav.searchClosestAngle(direction_angle, self.robotNav.getColumnFromList(2, minima))
        # closest_angle = self.computeClosestAngle(direction_angle, minima)

        hist = np.asarray(self.histogram)
        hist = hist.transpose()
        indexes= np.where(hist[1] < self.threshold)
        minima = hist[0][indexes]
        # search for closest angle
        closest_angle = Calc.search_closest_angle(direction_angle, minima.tolist())


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

        try:
            if self.robotNav.AngleInRange(
                    closest_angles[0], closest_angles[1], target_angle, True, self.angle_threshold):
                closest_angle = target_angle
            else:
                closest_angle = closest_angles[2]
        except IndexError or TypeError:
            print "error"

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

        minimum_found = False

        # recognizes if loop runs inside a minumum
        inside_minimum = False

        for i in range(len(histrogram)):

            # minimum starts
            if histrogram[i][1] < self.threshold and not inside_minimum:
                inside_minimum = True
                # save starting angle of minimum
                start_angle = histrogram[i][0]


            # minmum ends
            elif histrogram[i][1] > self.threshold and inside_minimum:
                inside_minimum = False
                # save end angle of minimum
                end_angle = histrogram[i-1][0]
                # add minimum to list
                minima.append([start_angle, end_angle, 0])
                # save that at least one minimum was found
                minimum_found = True

        # if minimum doesn't end at last entry
        if inside_minimum == True:
            # if this is the first minimum
            if not minimum_found:
                # end angle is last entry in list
                end_angle = histrogram[-1][0]
                # add the minimum
                minima.append([start_angle, end_angle, 0])
                # save that at least one minimum was found
                minimum_found = True

            # check if minimum goes on at first entry
            if histrogram[0][1]< self.threshold:
                # combine with first found minimum by start angle
                minima[0][0] = start_angle


        # calculate mid angles
        for i in range(len(minima)):
            minima[i][2] = self.robotNav.getMedialAngle(minima[i][0], minima[i][1])

        return minima




