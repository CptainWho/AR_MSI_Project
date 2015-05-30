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
    :param p: robot point
    :param p_target: target point
    :param tol: tolerance
    :return: True if in tol, False if not in tol
    """
    sqr_dist = (p[0] - p_target[0])**2 + (p[1] - p_target[1])**2
    sqr_tol = tol**2
    if sqr_dist < sqr_tol:
        return True
    else:
        return False