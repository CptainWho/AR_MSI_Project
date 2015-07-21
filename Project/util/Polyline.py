__author__ = 'Ecki'

from Project.util import Calculations as Calc

class Polyline:

    def __init__(self, polyline):
        self.polyline = None
        self.original_polyline = None
        self.tol_point = 0.1
        # distance between the to two actual points
        self.point_dist = None
        # init the polyline
        self.set_polyline(polyline)

    def set_polyline(self, polyline):
        """
        defines the polyline for robot to follow
        :param polyline:
        :return:
        """
        self.polyline = polyline
        self.original_polyline = polyline
        if self.polyline == None:
            self.point_dist = None
        else:
            self.point_dist = Calc.get_dist_from_point_to_point(self.p_last(), self.p_next())

    def set_point_tolerance(self, tol):
        self.tol_point = tol

    def get_dist_between_points(self):
        """
        returns the distance between the actual last point and next point
        :return:
        """
        return self.point_dist

    def p_end(self):
        return self.original_polyline[-1]

    def get_original_polyline(self):
        return self.original_polyline

    def get_point_tolerance(self):
        return self.tol_point

    def p_next(self):
        if len(self.polyline) < 2:
            return self.p_last()
        else:
            return self.polyline[1]

    def p_last(self):
        return self.polyline[0]

    def update_points(self):
        """
        switches next and last point
        :return: True if successful, False if polyline end of polyline is reached
        """
        if len(self.polyline) < 3:
            return False
        else:
            self.polyline.remove(self.polyline[0])
            self.point_dist = Calc.get_dist_from_point_to_point(self.p_last(), self.p_next())
            return True