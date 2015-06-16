# -*- coding: utf-8 -*-
""" Module TestDictionary
"""

__project__ = 'Exercise 3'
__module__  = 'TestDictionary'
__author__  = 'Philipp Lohrer'
__date__    = '14.06.2015'

__version__ = '0.1'

# Standard library imports
import numpy as np
# Local imports

# Example cells
cells = np.array([[0, 0], [1, 1], [3, 1], [5, 4], [7, 7]])

# Create dictionary with example cells and set priority of each cell to 0
open_list = dict([(tuple(cell), 0) for cell in cells])
closed_list = {}

# Add a new item to dic
closed_list[(3, 3)] = 1

print open_list
print closed_list

if (3, 3) not in closed_list and (3, 3) not in open_list:
    print 'Tuple not in either dic'
else:
    print 'Tuple in dic'

# Pop and print all items
while open_list:
    cell, d = open_list.popitem()
    print cell, d







