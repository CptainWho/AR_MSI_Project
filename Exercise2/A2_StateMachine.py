# -*- coding: utf-8 -*-
__author__ = 'Ecki, Philipp'
# State Machine from http://python-3-patterns-idioms-test.readthedocs.org/en/latest/StateMachine.html

from math import *
import numpy as np
import CarrotDonkey as cd

''' noch nicht fertig'''

class A2_StateMachine:
    def __init__(self, initial_state):
        self.currentState = initial_state
        self.currentState.run()

    def runAll(self, inputs):
        for i in inputs:
            print i
            self.currentState = self.currentState.next(i)
            return self.currentState.run()



    """ State Class """
class State:
    def __init__(self):
        pass

    def run(self):
        assert 0, "run not implemented"

    def next(self, input):
        assert 0, "next not implemented"