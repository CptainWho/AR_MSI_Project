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


def diff_abs(theta, theta_target, counterclock=True):
    """ Calculate the absolute angle difference from theta to theta_target
        positive is defined as counterclockwise
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
    angle = (theta_target - theta) % (2 * pi)
    if counterclock:
        return angle
    else:
        return angle - 2 * pi

def add_angles(angle1, angle2):
    """
    Add given angles
    :param angle1: -
    :param angle2: -
    :return: angle [-pi...pi]
    """
    return  (angle1 + angle2 + pi) % (2*pi) - pi

def add_angles_positive(angle1, angle2):
    """ Add given angles
    output is a positive angle
    :param angle1: -
    :param angle2: -
    :return: angle [0...2*pi]
    """

    return (angle1 + angle2) % (2*pi)

def get_angle_from_point_to_point(point1, point2):
    """
    returns the angle from point1 to point2 in global coordinate system
    :param point1:
    :param point2:
    :return:
    """
    theta_target = atan2(point2[1] - point1[1], point2[0] - point1[0])
    return theta_target

def get_medial_angle(start_angle, end_angle):
    """ Calculates angle in the middle of two given angles.
    the middle is always seen from the shortest distance between the two angles.
    :param start_angle: -
    :param end_angle: -
    :return: angle raging from -pi to pi
    """
    mid_angle = diff(start_angle, end_angle) / 2.0
    mid_angle += start_angle
    return mid_angle

def get_medial_angle_positive(start_angle, end_angle, counterclock = True):
    """ Calculates angle in the middle of two given angles.
    the middle is always seen from start_angle to end_angle in given direction
    :param start_angle: -
    :param end_angle: -
    :return: angle raging from 0 to 2*pi
    """
    diff = diff_abs(start_angle, end_angle, counterclock)
    mid_angle = add_angles_positive(start_angle, diff/2.0)
    return mid_angle

def search_closest_angle(target_angle, list_of_angles):
    """ Searches inside given list of angles for the closest one and returns it
    :param target_angle: -
    :param list_of_angles: array-like
    :return: closest angle
    """

    if list_of_angles is None:
        return target_angle

    # Convert list/array to np.array
    list_of_angles = np.asarray(list_of_angles)

    closest_angle = 0
    diff_old = 2.0 * pi
    for angle in np.nditer(list_of_angles):
        diff_temp = diff_abs(angle, target_angle)
        if diff_old > diff_temp:
            diff_old = diff_temp
            closest_angle = angle
    return closest_angle


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
    if 2.0 * offset > diff_abs(start_angle, end_angle, counterclock):
        return False

    # check for range
    if counterclock:
        if diff_abs(start, angle) <= diff_abs(start, end):
            return True
        else:
            return False
    else:
        if diff_abs(start, angle) >= diff_abs(start, end):
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

# def get_angle_from_robot_to_point(robot_pos, point):
#     """ Returns the angle difference between the robot orientation and given point
#     :param robot_pos: [x,y,theta]
#     :param point: point (x,y)
#     :return: angle difference
#     """
#
#     theta_robot = robot_pos[2]
#     theta_target = get_angle_from_robot_to_point(robot_pos, point)
#     return diff(theta_robot, theta_target)

# def get_angle_from_robot_to_point(robot_pos, point):
#     """ Returns angle between robot position and a point
#     :param point: [x,y]
#     :return: angle [0..2*pi]
#     """
#
#     [x, y, theta] = robot_pos
#     theta_target = atan2(point[1] - y, point[0] - x)
#     if theta_target < 0:
#         theta_target += 360.0
#     return theta_target