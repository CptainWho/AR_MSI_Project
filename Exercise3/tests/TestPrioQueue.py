# -*- coding: utf-8 -*-
""" Module TestPrioQueue
"""

__project__ = 'Exercise 3'
__module__  = 'TestPrioQueue'
__author__  = 'Philipp Lohrer'
__date__    = '14.06.2015'

__version__ = '0.1'

# Standard library imports
from Queue import PriorityQueue
# Local imports

prio_queue = PriorityQueue()

# ._put(prio1,prio2,..,prio_n,item)
prio_queue._put((1,1,3,'1,2'))
prio_queue._put((1,1,2,'1,1'))
prio_queue._put((1,4,'1,4'))
prio_queue._put((2,0,'2,0'))
prio_queue._put((2,2,'2,2'))
prio_queue._put((3,2,'3,2'))

while not prio_queue.empty():
    item = prio_queue._get()
    print item
