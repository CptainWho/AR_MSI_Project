# -*- coding: utf-8 -*-
""" Module TestWeightedChoise
"""

__project__ = 'Exercise 4'
__module__  = 'TestWeightedChoise'
__author__  = 'Philipp Lohrer'
__date__    = '25.06.2015'

__version__ = '0.1'

# Standard library imports
import numpy as np
import random
import bisect
from datetime import datetime
# Local imports

class WeightedRandomGenerator(object):
    """
    source: http://eli.thegreenplace.net/2010/01/22/weighted-random-generation-in-python
    """

    def __init__(self, weights):
        self.totals = []
        running_total = 0

        for w in weights:
            running_total += w
            self.totals.append(running_total)

    def next(self):
        rnd = random.random() * self.totals[-1]
        return bisect.bisect_right(self.totals, rnd)

    def __call__(self):
        return self.next()


def weighted_random_generator(elements, weights, n):
    t_start = datetime.now()

    wrg = WeightedRandomGenerator(weights)

    one = 0
    two = 0
    three = 0
    four = 0

    for i in xrange(n):
        element = elements[wrg()]
        if element == 'one':
            one += 1
        elif element == 'two':
            two+= 1
        elif element == 'three':
            three += 1
        elif element == 'four':
            four += 1

    t_delta = (datetime.now() - t_start).total_seconds()

    print 'Weighted Random Generator:'
    print '\tone: %d, two: %d, three: %d, four: %d' % (one, two, three, four)
    print '\t%d elements %d-times in %0.2f seconds' % (len(elements), n, t_delta)


def numpy_weighted_choice(elements, weights, n):

    t_start = datetime.now()

    one = 0
    two = 0
    three = 0
    four = 0

    for i in xrange(n):
        element = np.random.choice(elements, p=weights)
        if element == 'one':
            one += 1
        elif element == 'two':
            two+= 1
        elif element == 'three':
            three += 1
        elif element == 'four':
            four += 1

    t_delta = (datetime.now() - t_start).total_seconds()

    print 'Numpy weighted choice:'
    print '\tone: %d, two: %d, three: %d, four: %d' % (one, two, three, four)
    print '\t%d elements %d-times in %0.2f seconds' % (len(elements), n, t_delta)



elements = ['one', 'two', 'three', 'four']
weights = [0.15, 0.25, 0.5, 0.1]
n = 100000

# numpy_weighted_choice(elements, weights, n)

weighted_random_generator(elements, weights, n)
