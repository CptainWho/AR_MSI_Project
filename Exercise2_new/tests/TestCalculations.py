__author__ = 'Ecki'

from math import *
from Exercise2_new.util import Calculations as Calc

p1 = [0, 0]
p2 = [1, -0.01]

angle = Calc.get_angle_of_line(p1, p2)
angle_pos = Calc.get_positive_angle_of_line(p1,p2)

print angle*180/pi
print angle_pos*180/pi
