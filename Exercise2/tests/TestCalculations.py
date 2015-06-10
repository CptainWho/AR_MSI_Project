__author__ = 'Ecki'

from math import *
from Exercise2.util import Calculations as Calc

angle1 = -0.69813170079773179 #-10*pi/180
angle2 = -0.52359877559829882+pi#90*pi/180

target = 5.6723200689815707

#[-0.69813170079773179, -0.52359877559829882, 5.6723200689815707]

print angle1*180/pi
print angle2*180/pi
print Calc.get_medial_angle_positive(angle1, angle2, False)*180/pi
print Calc.get_medial_angle(angle1, angle2, False)*180/pi