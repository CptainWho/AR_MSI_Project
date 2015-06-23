# -*- coding: utf-8 -*-
""" Module MCL
"""

__project__ = 'Exercise 4'
__module__  = 'MCL'
__author__  = 'Philipp Lohrer'
__date__    = '21.06.2015'

__version__ = '0.1'

# Standard library imports
from math import pi
import random as rnd
# Local imports



def MCL(particle_cloud, movement, sensor_data):

    for particle in particle_cloud:
        # Move particle according to movement
        particle()
