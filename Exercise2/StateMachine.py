# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from math import *

class StateMachine:
    def __init__(self, my_robot, transitions):
        self.robot = my_robot
        self.transitions = transitions
        self.states = {'NoObstacle', 'Obstacle', 'CornerReached', 'TargetReached'}
        self.current_state = 'CornerReached'


    # determine next state
    def next_state(self):
        if self.current_state in self.states:
            if self.current_state == 'NoObstacle':
                if self.transitions.obstacleInSight():
                    self.current_state = 'Obstacle'
                    print self.current_state
                if self.transitions.nextPointReached():
                    if self.transitions.endPointReached():
                        self.current_state = 'TargetReached'
                    else:
                        self.current_state = 'CornerReached'

            if self.current_state == 'Obstacle':
                if not self.transitions.obstacleInSight():
                    self.current_state = 'NoObstacle'
                    print self.current_state
                if self.transitions.nextPointReached():
                    if self.transitions.endPointReached():
                        self.current_state = 'TargetReached'
                    else:
                        self.current_state = 'CornerReached'

            if self.current_state == 'CornerReached':
                if self.transitions.aimingToNextPoint():
                    self.current_state = 'NoObstacle'

            if self.current_state == 'TargetReached':
                pass

        return self.current_state