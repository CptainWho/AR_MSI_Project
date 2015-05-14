# -*- coding: utf-8 -*-
__author__ = 'Ecki, Philipp'
# State Machine from http://python-3-patterns-idioms-test.readthedocs.org/en/latest/StateMachine.html

from math import *
import numpy as np
import CarrotDonkey as cd

''' noch nicht fertig'''

class A2_StateMachine:
    def __init__(self, my_robot, my_world, initial_state):
        self.robot = my_robot
        self.currentState = initial_state
        self.currentState.run()
        myCD = cd.CarrotDonkey(my_robot, my_world)

    def applyStates(self, inputs):
        for i in inputs:
            print i
            self.currentState = self.currentState.next(i)
            return self.currentState.run()

    """ State Class """
    class State:
        def __init__(self):
            pass

        @staticmethod
        def run(self):
            assert 0, "run not implemented"

        @staticmethod
        def next(self, input):
            assert 0, "next not implemented"

        """ States """
    class NoObstacle:
        def __init__(self):
            pass

        @staticmethod
        def run(self):
            [v, omega] =
            return [v, omega]

        @staticmethod
        def next(self, input):


    class Obstacle:
        def __init__(self):
            pass

        def run(self):
            return 2


    class RotateToNext:
        def __init__(self):
            pass