__author__ = 'Ecki'

from math import *
from Exercise2_new.util import Calculations as Calc

angle1 = -1.57079632679 #-10*pi/180
angle2 = 4.53785605519#90*pi/180

target = -0.0366485445028

print angle1*180/pi
print angle2*180/pi
print target*180/pi
print Calc.angle_in_range(angle1, angle2, target)