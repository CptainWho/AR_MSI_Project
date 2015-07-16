""" Module Description:
Contains basic calculations
"""

__author1__ = 'Ecki'
__author2__ = 'Philipp Lohrer'
__date__ = '30.05.2015'
__version__ = '1.0'

# Standard library imports
from math import *
import numpy as np
# Local imports


def polar_2_cartesian(radius, theta):
    """ Convert given polar coordinates to cartesian coordinates
    :param radius: -
    :param theta: -
    :return: cartesian coordinates x,y
    """

    x = radius * cos(theta)
    y = radius * sin(theta)

    return [x, y]


def cartesian_2_polar(x, y):
    """ Convert given cartesian coordinates to polar coordinates
    :param x:
    :param y:
    :return:
    """

    # TODO


def diff(theta, theta_target):
    """ Calculate the angle difference from theta to theta_target
        positive is defined as counterclockwise
    :param theta: angle robot
    :param theta_target: angle target
    :return: angle difference
    """

    return (theta_target - theta + pi) % (2 * pi) - pi


def point_in_tol(p, p_target, tol):
    """ Check whether point in within tolerance of target point
    :param p: point [x1,y1]
    :param p_target: target point [x2,y2]
    :param tol: tolerance
    :return: True if in tol, False if not in tol
    """

    sqr_dist = (p[0] - p_target[0])**2 + (p[1] - p_target[1])**2
    sqr_tol = tol**2
    if sqr_dist < sqr_tol:
        return True
    else:
        return False


def angle_in_tol(angle, angle_target, tol):
    """ Check whether angle is within tolerance of target angle
    :param angle: -
    :param angle_target: -
    :param tol:  tolerance
    :return: True if in tol, False if not in tol
    """

    angle_diff = abs(diff(angle, angle_target))
    if angle_diff < tol:
        return True
    else:
        return False


def diff_custom(start_angle, end_angle, counterclock=True,):
    """ Calculate the angle difference from theta to theta_target
        positive is defined as counterclockwise (output from 0 to pi)
        If counterclock is false rotation direction is changed (output from 0 to -2*pi)
    :param theta: -
    :param theta_target: -
    :param counterclock: default = True
    :return: angle [0...2*pi]
    """
    # make angles positive
    #theta %= 2 * pi
    #theta_target %= 2 * pi

    # calculate difference
    angle = (end_angle - start_angle) % (2 * pi)
    if counterclock:
        return angle
    else:
        return angle - 2 * pi


def diff_abs(theta, theta_target):
    """ Calculate the absolute angle difference from theta to theta_target
        positive is defined as counterclockwise
    :param theta: -
    :param theta_target: -
    :return: angle [0...pi]
    """

    angle = (theta_target - theta + pi) % (2 * pi) - pi
    # if theta < theta_target:
    #     # Counterclock-wise
    #     return abs(angle)
    # else:
    #     # Clockwise
    #     return abs(angle - 2 * pi)

    return abs(angle)


def add_angles(angle1, angle2):
    """ Add given angles
    :param angle1: -
    :param angle2: -
    :return: angle [-pi...+pi]
    """

    return (angle1 + angle2 + pi) % (2*pi) - pi


def get_angle_from_point_to_point(start_point, end_point):
    """
    returns angle between two points
    :param start_point:
    :param end_point:
    :return: angle theta [-pi...0...+pi]
    """
    theta = atan2(end_point[1] - start_point[1], end_point[0] - start_point[0])
    return theta


def get_dist_from_point_to_point(start_point, end_point):
    """
    returns distance between two points
    :param start_point:
    :param end_point:
    :return:
    """
    dist = sqrt((end_point[1] - start_point[1])**2 + (end_point[0] - start_point[0])**2)
    return dist


def get_medial_angle(start_angle, end_angle, counterclock=True):
    """ Calculates angle in the middle of two given angles.
    the middle is always seen from start_angle to end_angle in given direction
    :param start_angle: -
    :param end_angle: -
    :return: angle raging from -pi to pi
    """
    mid_angle = get_medial_angle_custom(start_angle, end_angle, counterclock)
    mid_angle = (mid_angle + pi) % (2 * pi) - pi
    return mid_angle


def get_medial_angle_custom(start_angle, end_angle, counterclock=True):
    """ Calculates angle in the middle of two given angles.
    the middle is always seen from start_angle to end_angle in given direction
    :param start_angle: -
    :param end_angle: -
    :return: angle raging from 0 to 2*pi
    """
    diff = diff_custom(start_angle, end_angle, counterclock)
    mid_angle = add_angles_positive(start_angle, diff/2.0)
    return mid_angle


def get_average_angle(list_angles):
    """ Calculate average angle of all given angles and return it.
    For this the unit vectors of all angles are computed and summed.
    If average_angle is not defined (x,y = 0) return None
    :param list_angles: (list) angles (0..2*pi)
    :return:            average angle (0..2*pi) or None if average_angle not defined
    """

    x, y = [0, 0]
    for angle in list_angles:
        x += cos(angle)
        y += sin(angle)
    if x == 0 and y == 0:
        return None
    average_angle = atan2(y, x)
    return average_angle % (2 * pi)


def search_closest_angle(target_angle, list_of_angles):
    """ Searches inside given list of angles for the closest one and returns it
    :param target_angle: -
    :param list_of_angles: array-like
    :return: closest angle, angle_diffs
    """

    # TODO Optimize search

    if list_of_angles is None:
        return target_angle

    # Convert list/array to np.array
    list_of_angles = np.asarray(list_of_angles)

    closest_angle = 0
    angle_diffs = np.empty(0, dtype=np.float)
    diff_old = 2.0 * pi
    for angle in np.nditer(list_of_angles):
        diff_temp = diff_abs(angle, target_angle)
        if diff_old > diff_temp:
            diff_old = diff_temp
            closest_angle = angle
        # Add current angle diff to array
        angle_diffs = np.hstack((angle_diffs, diff_temp))
    return closest_angle, angle_diffs


def add_angles_positive(angle1, angle2):
    """ Add given angles
    output is a positive angle
    :param angle1: -
    :param angle2: -
    :return: angle [0...2*pi]
    """

    return (angle1 + angle2) % (2*pi)


def angle_in_range(start_angle, end_angle, angle, counterclock=True, offset=0):
    """ Checks if given angle lies between two angles,
        the direction is defined from start_angle to end_angle in given rotation direction
    :param start_angle: -
    :param end_angle: -
    :param angle: angle to look for
    :param counterclock: True / False
    :param offset:
    :return: True / False
    """

    # add offset
    start = add_angles_positive(start_angle, offset)
    end = add_angles_positive(end_angle, -offset)

    # return false if offset is too big
    if 2.0 * offset > diff_custom(start_angle, end_angle, counterclock):
        return False

    # check for range
    if counterclock:
        if diff_custom(start, angle) <= diff_custom(start, end):
            return True
        else:
            return False
    else:
        if diff_custom(start, angle) >= diff_custom(start, end):
            return True
        else:
            return False


def get_angle_of_line(p1, p2):
    """
    returns the angle of the line in global coordinate system
    :param p1: line start
    :param p2: line end
    :return: angle from -pi to pi
    """
    theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
    return theta


def get_positive_angle_of_line(p1, p2):
    """
    returns positive the angle of the line in global coordinate system
    :param p1: line start
    :param p2: line end
    :return: angle from 0 to 2*pi
    """
    theta = get_angle_of_line(p1, p2) % (2*pi)
    return theta

def limit_value(value, limit):
    """
    limit a value to a certain limit
    e.g. if limit = 1, the value is limited from -1...+1
    :param value:
    :param limit:
    """
    abs_limit = abs(limit)
    if value > abs_limit:
        value = abs_limit
    if value < -abs_limit:
        value = -abs_limit
    return value

def get_projected_distance_on_line(start, end, point):
    """
    calculates the distance of a_b from picture at:
    http://www.gymbase.de/index/themeng13/ma/orthogonal_03.php
    :param start:
    :param end:
    :param point:
    :return:
    """
    # calculate line distances
    line_del_x = end[0] - start[0]
    line_del_y = end[1] - start[1]
    norm_a = get_dist_from_point_to_point(start, end)
    vect_a = np.asarray([line_del_x, line_del_y])
    vect_b = np.asarray([point[0]-start[0], point[1]-start[1]])

    a_b = np.dot(vect_a, vect_b) / float(norm_a)

    return a_b

def get_orthogonal_point_on_line(line_start, line_end, point, offset=0):
    """
    returns a point that lies on the line and is orthogonal to the given point
    :param line_start:
    :param line_end:
    :param point:
    :return:
    """
    dist_new = get_projected_distance_on_line(line_start, line_end, point)
    dist_old = get_dist_from_point_to_point(line_start, line_end)
    line_del_x = line_end[0] - line_start[0]
    line_del_y = line_end[1] - line_start[1]
    x_new = line_del_x * (dist_new + offset) / dist_old
    y_new = line_del_y * (dist_new + offset) / dist_old
    p_new = [x_new, y_new]
    return p_new

def get_signed_distance_from_line_to_point(start, end, point):
    """
    calculates the distance of the dashed line from picture at:
    http://www.gymbase.de/index/themeng13/ma/orthogonal_03.php
    positive is on the left side of the line when you stand on start and look to end
    :param start:
    :param end:
    :param point:
    :return:
    """
    # calculate line distances
    line_del_x = end[0] - start[0]
    line_del_y = end[1] - start[1]
    line_len = get_dist_from_point_to_point(start, end)

    # orthogonal vector on line
    ortho_vect = np.asarray([-line_del_y, line_del_x])
    # vector from start to point
    point_vect = np.asarray([point[0] - start[0], point[1] - start[1]])

    distance = np.dot(ortho_vect, point_vect) / float(line_len)
    return distance

def douglas_peucker(polyline, epsilon):
    """
    algorithm for reducing the number of points in a curve that is approximated by a series of points
    https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
    :param polyline: the polyline to simplify
    :param epsilon: tolerance factor
    :return: simplified polyline
    """
    # if polyline empty do nothing
    if len(polyline) < 2:
        return []

    # Find the point with the maximum distance
    d_max = 0
    index = 0
    for i in range(1, len(polyline)-1):
        point = polyline[i]
        d = abs(get_signed_distance_from_line_to_point(polyline[0], polyline[-1], point))
        if d > d_max:
            index = i
            d_max = d

    # If max distance is greater than epsilon, recursively simplify
    if d_max >= epsilon:
        # Recusive call
        rec_results_1 = douglas_peucker(polyline[0:index], epsilon)
        rec_results_2 = douglas_peucker(polyline[index:len(polyline)], epsilon)

        # Build the result list
        result_list = rec_results_1[0:len(rec_results_1)-1]
        result_list.extend(rec_results_2[0:len(rec_results_2)])

    else:
        result_list = [polyline[0], polyline[-1]]

    # Return the result
    return result_list

# def get_closest_line_to_point(polyline, target_point):
#     """
#     returns the closest line
#     :param point_list:
#     :return: [first, second]
#     """
#     # when list is too small
#     if len(point_list) < 2:
#         return [point_list[0], point_list[0]]
#
#     # closest distance
#     first_dist = float("inf")
#     # second closest distance
#     second_dist = float("inf")
#
#     # go though all points
#     for i in xrange(len(point_list)):
#         dist = get_dist_from_point_to_point(point_list[i], target_point)


# OBSOLET
# # returns selected column from list (beginning at 0)
# def getColumnFromList(self, column_number, list):
#
#     newlist = []
#     try:
#         for i in list:
#             number = i[column_number]
#             newlist.append(number)
#         return newlist
#     except TypeError:
#         return None

