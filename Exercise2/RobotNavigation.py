# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *


class RobotNavigation:

    def __init__(self, robot):
        self.robot = robot
        self.polyline = [self.getRobotPoint(), self.getRobotPoint()]
        self.tolPoint = 0.1
        self.tolAngle = 0.1

    # defines the polyline for robot to follow
    def setPolyline(self, polyline):
        self.polyline = polyline

    def getNextPoint(self):
        return self.polyline[1]

    def getLastPoint(self):
        return self.polyline[0]

    # check if robot has reached a point and updates to next point
    def nextPointReached(self):
        if self.indsideTol(self.getNextPoint(), self.tolPoint):
            if len(self.polyline) > 2:
                self.polyline.remove(self.polyline[0])
            return True
        else:
            return False

    # check if robot directly looks to the next point
    def aimingToNextPoint(self):
        if self.insideAngleTol(self.getAngleToPoint(self.getNextPoint()), self.tolAngle):
            return True
        else:
            return False

    # check if End Point is reached
    def endPointReached(self):
        if len(self.polyline) == 2:
            if self.indsideTol(self.getNextPoint(), self.tolPoint):
                return True
            else:
                return False
        else:
            return False

    # returns the robots position
    def getRobotPos(self):
        return self.robot.getTrueRobotPose()

    # returns robots angle in global coordinate system
    def getRobotAngle(self):
        [x, y, theta] = self.getRobotPos()
        return theta

    # returns robots x and y value in global coordinate system
    def getRobotPoint(self):
        [x, y, theta] = self.getRobotPos()
        return [x, y]

    # check whether robot in within tolerance of target point
    def indsideTol(self, p_target, tol):
        p = self.getRobotPoint()
        sqrDist = (p[0] - p_target[0])**2 + (p[1] - p_target[1])**2
        sqrTol = tol**2
        if(sqrDist > sqrTol):
            return False
        else:
            return True

    # check whether angle is within tolerance of target angle
    def insideAngleTol(self, angle_target, tol):
        angle = self.getRobotAngle()
        angle_diff = abs(self.diff(angle, angle_target))
        if angle_diff > tol:
            return False
        else:
            return True

    # calculate the angle difference from theta to theta_target
    # positive is defined as counterclockwise
    def diff(self, theta, theta_target):
        return (theta_target - theta + pi) % (2 * pi) - pi

    # returns the angle between robot orientation and a point
    def getAngleToPoint(self, point):
        # target direction
        [x, y, theta] = self.getRobotPos()
        theta_target = atan2(point[1] - y, point[0] - x)
        return theta_target

    # decides whether there is a obstacle in robots line of sight
    def obstacleInSight(self):
        sensorDist = self.robot.sense()
        # check if there is a obstacle in front of robot
        for distance in sensorDist[5:11]:
            if distance != None:
                return True
        return False

    # calculates angle in the middle of two given angles
    def getMedialAngle(self, start_angle, end_angle):
        mid_angle = self.diff(start_angle, end_angle)/2.0
        mid_angle += start_angle
        return mid_angle


    # searches inside the list of angles for the closest one and returns it
    def searchClosestAngle(self, target_angle, list_of_angles):
        closest_angle = 0
        diff_old = 2*pi
        for angle in list_of_angles:
            diff = abs(self.diff(angle, target_angle))
            if diff_old > diff:
                diff_old = diff
                closest_angle = angle
        return closest_angle

    # returns selected column from list (beginning at 0)
    def getColumnFromList(self, column_number, list):
        newlist = []
        for i in list:
            newlist.append(i[column_number])
        return newlist