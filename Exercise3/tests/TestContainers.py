# -*- coding: utf-8 -*-
""" Module TestContainer
"""

__project__ = 'Exercise 3'
__module__  = 'TestContainer'
__author__  = 'Philipp Lohrer'
__date__    = '17.06.2015'

__version__ = '1.0'

# Standard library imports
import numpy as np
import heapq as hq
import matplotlib.pyplot as plt
# Local imports


class OpenList():
    """ Class description:
    Creates a container which acts as an open list and is internally sorted after the priority of added items
    """

    def __init__(self, values=None):
        if values is None:
            self.values = []
        else:
            self.values = values

    def __len__(self):
        return len(self.values)

    def __contains__(self, key):
        return True if key in self.values else False

    # def __getitem__(self, key):
    #     return self.values[key]
    #
    # def __setitem__(self, key, value):
    #     self.values[key] = value
    #
    # def __delitem__(self, key):
    #     del self.values[key]

    def __iter__(self):
        return iter(self.values)

    def pop(self):
        hq.heapify(self.values)
        return hq.heappop(self.values)




ol = OpenList([1,2,3,4,6,0.5])
if 2 in ol:
    print 'in there'
else:
    print 'false'

print ol.pop()
print len(ol)

for value in ol:
    print value