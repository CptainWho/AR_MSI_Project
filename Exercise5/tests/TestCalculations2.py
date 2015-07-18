__author__ = 'Ecki'

from Exercise5.util import Calculations as Calc
from math import *

min_angles = [-190*pi/180.0, 15*pi/180.0, 7*pi/180.0]
target_angle = 9*pi/180.0

print Calc.search_closest_angle(target_angle, min_angles)[0]*180/pi