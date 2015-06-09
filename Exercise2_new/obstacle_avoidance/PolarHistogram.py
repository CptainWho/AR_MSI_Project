# -*- coding: utf-8 -*-
__author__ = 'Ecki'

import numpy as np
from math import *
from Exercise2_new.util import Calculations as Calc

class PolarHistogram:

    def __init__(self, robot, robot_location):
        self.robot = robot
        self.robot_loc = robot_location
        self.threshold = 0.05
        self.angle_threshold = 10*pi/180
        # angle distance the robot keeps away from an edge
        self.angle_edge_offset = 20*pi/180
        self.histogram = None
        self.k_p_omega = 0.8
        # maximum distance for object recognition
        self.detection_dist = self.robot.getMaxSenseValue()/2.0

    def avoid_obstacle(self, target_point):
        direction_angle = self.robot_loc.get_angle_from_robot_to_point(target_point)
        self.histogram = self.generateHistFromSensors()
        minima = self.locate_minima(self.histogram)
        #closest_angle = self.robotNav.searchClosestAngle(direction_angle, self.robotNav.getColumnFromList(2, minima))
        closest_angle = self.compute_closest_angle(direction_angle, minima)

        closest_angle_grad = closest_angle *180/pi
        if abs(closest_angle_grad) > 120:
            print closest_angle_grad


        # hist = np.asarray(self.histogram)
        # hist = hist.transpose()
        # indexes= np.where(hist[1] < self.threshold)
        # minima = hist[0][indexes]
        # # search for closest angle
        # closest_angle = Calc.search_closest_angle(direction_angle, minima.tolist())

        omega = self.k_p_omega * closest_angle

        # when bigger than max value
        if abs(omega) > self.robot.getMaxOmega():
            omega = np.sign(omega) * self.robot.getMaxOmega()


        #v = self.robot._maxSpeed * 1 * (1 - omega/self.robot._maxOmega)
        v = self.robot.getMaxSpeed()/2.0 * (1 - (omega/float(self.robot.getMaxOmega())))
        return [v, omega]

    def compute_closest_angle(self, target_angle, minima):
        """
        searches for closest angle in minima array
        :param target_angle:
        :param minima:
        :return:
        """
        diff_old = 2*pi
        # choose one minima from all minimas
        for angles in minima:
            start_angle = angles[0]
            end_angle = angles[1]
            mid_angle = angles[2]
            diff_mid = abs(Calc.diff(mid_angle, target_angle))
            diff_start = abs(Calc.diff(start_angle, target_angle))
            diff_end = abs(Calc.diff(end_angle, target_angle))

            #for diff in [diff_start, diff_end]:
            if diff_old > diff_mid:
                chosen_minima = angles
                diff_old = diff_mid

        start_angle = chosen_minima[0]
        end_angle = chosen_minima[1]
        mid_angle = chosen_minima[2]
        diff_start = abs(Calc.diff(start_angle, target_angle))
        diff_end = abs(Calc.diff(end_angle, target_angle))

        # if target is within threshold, return target
        if Calc.angle_in_range(
                start_angle, end_angle, target_angle, True, self.angle_threshold):
            return target_angle

        # when minima window is too small, choose middle
        if Calc.diff_custom(start_angle, end_angle) < 2*self.angle_edge_offset:
            return mid_angle

        # check edge where the target is close and add offset
        if diff_start < diff_end:
            closest_angle = Calc.add_angles(start_angle, self.angle_edge_offset)
        else:
            closest_angle = Calc.add_angles(end_angle, -self.angle_edge_offset)

        return closest_angle

    def generateHistFromSensors(self):
        """
        generates a dicrete histogram from robots sensor data
        :return:
        the histogram
        """

        #print(sensor_data)
        sensor_values = self.robot.sense()
        for i in range(len(sensor_values)):
            if sensor_values[i] > self.detection_dist or sensor_values[i] is None:
                sensor_values[i] = 0.0
            else:
                sensor_values[i] = self.detection_dist - sensor_values[i]
                if sensor_values[i] < 0:
                    sensor_values[i] = 0.0

        sensor_data = np.column_stack((self.robot._sensorDirections, sensor_values))
        return sensor_data

    # generates the 2D-List mininma
    # each minimum is described in one line with the colums: start, end, mid angle
    def locate_minima(self, histrogram):
        minima = []
        start_angle = 0

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
            #minima[i][2] = Calc.get_medial_angle(minima[i][0], minima[i][1])
            minima[i][2] = Calc.get_medial_angle(minima[i][0], minima[i][1])

        return minima



# OBSOLETE
#     def compute_closest_angle(self, target_angle, minima):
#         """
#         searches for closest angle in minima array
#         :param target_angle:
#         :param minima:
#         :return:
#         """
#         closest_angles = []
#         diff_old = 2*pi
#         for angles in minima:
#             diff = abs(Calc.diff(angles[2], target_angle))
#             if diff_old > diff:
#                 diff_old = diff
#                 closest_angles = angles
#
#         if Calc.angle_in_range(
#                 closest_angles[0], closest_angles[1], target_angle, True, self.angle_threshold):
#             closest_angle = target_angle
#         else:
#             closest_angle = closest_angles[2]
#
#         return closest_angle
