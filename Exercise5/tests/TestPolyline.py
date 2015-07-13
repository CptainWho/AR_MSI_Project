__author__ = 'Ecki'

from Exercise5.util import Polyline

line = [[1, 1], [2, 2], [3, 3], [4, 4]]
line = [[1, 1], [2, 2]]

polyline = Polyline.Polyline(line)

print polyline.update_points()
print polyline.update_points()

print polyline.p_last()
print polyline.p_next()