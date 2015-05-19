# -*- coding: utf-8 -*-
__author__ = 'Ecki'
# State Machine from http://python-3-patterns-idioms-test.readthedocs.org/en/latest/StateMachine.html

from math import *
import numpy as np


""" State Class """
class State:
    def __init__(self):
        pass

    def next(self, transitions):
        assert 0, "next not implemented"


""" State Machine """
class StateMachine:
    def __init__(self):

        self.no_obstacle = NoObstacle()
        self.obstacle = Obstacle()
        self.corner_reached = CornerReached()
        self.target_reached = TargetReached()
        self.currentState = self.no_obstacle

    def nextState(self, transitions):
        self.currentState = self.currentState.next(transitions)

    def stateEquals(self, State):
        if self.currentState == State:
            return True
        else:
            return False


""" State Transitions """


class NoObstacle(State):
    def next(self, transitions):
        if transitions.obstacleInSight():
            return Obstacle
        if transitions.nextPointReached():
            if transitions.endPointReached():
                return TargetReached
            else:
                return CornerReached
        return NoObstacle


class Obstacle(State):
    def next(self, transitions):
        if not transitions.obstacleInSight():
            return NoObstacle
        return Obstacle


class CornerReached(State):
    def next(self, transitions):
        if not transitions.aimingToNextPoint():
            return NoObstacle
        return CornerReached


class TargetReached(State):
    def next(self, transitions):
        return TargetReached