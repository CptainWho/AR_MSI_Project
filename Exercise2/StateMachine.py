# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *

class StateMachine:
    def __init__(self, my_robot):
        self.robot = my_robot
        self.states = {'NoObstacle', 'Obstacle', 'CornerReached', 'TargetReached'}
        self.current_state = 'CornerReached'
        self.polyline = [self.getRobotPoint(), self.getRobotPoint()]

    def getNextPoint(self):
        return self.polyline[1]

    def getLastPoint(self):
        return self.polyline[0]

    # decides whether there is a obstacle in robots line of sight
    def obstacleInSight(self):
        sensorDist = self.robot.sense()
        # check if there is a obstacle in front of robot
        for distance in sensorDist[5:11]:
            if distance != None:
                return True

        return False

    def getRobotPos(self):
        return self.robot.getTrueRobotPose()

    def getRobotAngle(self):
        [x, y, theta] = self.getRobotPos()
        return theta

    def getRobotPoint(self):
        [x, y, theta] = self.getRobotPos()
        return [x, y]

    # defines a polyline for robot to follow
    def setPolyline(self, polyline):
        self.polyline = polyline
        self.polyline.insert(0, self.getRobotPoint())

    # adds a additional point to the polyline
    def addPointToPolyline(self, point):
        self.polyline.append(point)

    # check whether point in within tolerance of target point
    def outOfTol(self, p, p_target, tol):
        sqrDist = (p[0] - p_target[0])**2 + (p[1] - p_target[1])**2
        sqrTol = tol**2
        if(sqrDist > sqrTol):
            return True
        else:
            return False

    # check whether angle is within tolerance of target angle
    def outOfAngleTol(self, angle, angle_target, tol):
        angle_diff = abs(self.diff(angle, angle_target))
        if angle_diff > tol:
            return True
        else:
            return False

    # calculate the angle difference from theta to theta_target
    # positive is defined as counterclockwise
    def diff(self, theta, theta_target):
        return (theta_target - theta + pi) % (2 * pi) - pi

    def getAngleToPoint(self, point):
        # target direction
        [x, y, theta] = self.getRobotPos()
        theta_target = atan2(point[1] - y, point[0] - x)
        return theta_target

    # determine next state
    def next_state(self):
        if self.current_state in self.states:
            if self.current_state == 'NoObstacle':
                if self.obstacleInSight():
                    self.current_state = 'Obstacle'
                    print self.current_state
                if not self.outOfTol(self.getRobotPoint(), self.getNextPoint(), 0.1):
                    if self.getNextPoint() == self.polyline[-1]:
                        self.current_state = 'TargetReached'
                    else:
                        self.current_state = 'CornerReached'
                        self.polyline.remove(self.polyline[0])

            if self.current_state == 'Obstacle':
                if not self.obstacleInSight():
                    self.current_state = 'NoObstacle'
                    print self.current_state

            if self.current_state == 'CornerReached':
                if not self.outOfAngleTol(self.getRobotAngle(), self.getAngleToPoint(self.getNextPoint()), 0.01):
                    self.current_state = 'NoObstacle'

            if self.current_state == 'TargetReached':
                pass

        return self.current_state