__author__ = 'Ecki'

from math import *
from Exercise2.util import Calculations as Calc

angle1 = -10*pi/180
angle2 = -11*pi/180

target = 110*pi/180

#[-0.69813170079773179, -0.52359877559829882, 5.6723200689815707]

print angle1*180/pi
print angle2*180/pi
print target*180/pi
print Calc.angle_in_range(angle1, angle2, target, True)
