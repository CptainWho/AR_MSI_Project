# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from Exercise5.util import Calculations as Calc

start_point = [0, 0]
end_point = [4.5, 1.5]

point = [2, -1]
#point = [3, -0.5]

print Calc.get_orthogonal_point_on_line(start_point, end_point, point)
print Calc.get_signed_distance_from_line_to_point(start_point, end_point, point)

print Calc.get_orthogonal_point_on_line(start_point, end_point, point, offset=-1.6)