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
# Local imports

one = 0
two = 0
three = 0

elements = ['one', 'two', 'three']
weights = [0.2, 0.3, 0.5]

for i in xrange(100):
    element = np.random.choice(elements, p=weights)
    if element == 'one':
        one +=1
    elif element == 'two':
        two+=1
    elif element == 'three':
        three +=1

print 'one: %d, two: %d, three: %d' % (one, two, three)