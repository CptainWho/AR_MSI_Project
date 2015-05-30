# -*- coding: utf-8 -*-
""" Module Description:
Tests for class HistogramGrid
"""

__project__ = 'Exercise 2'
__module__ = 'TestHistogram'
__author__ = 'Philipp Lohrer'
__email__ = 'plohrer@htwg-konstanz.de'
__date__ = '30.05.2015'

__version__ = '0.1'

# Standard library imports
from math import pi
# Local imports
from Exercise2_new.obstacle_avoidance import HistogramGrid


def test_grid_generation(hg):
    """ Fill histogram with values
    :param hg: histogram reference
    :return: -
    """
    hg.set_value(2, 0, debug=True)
    hg.set_value(2, pi/4, debug=True)
    hg.set_value(3, pi/4, debug=True)
    hg.set_value(2, pi/2, debug=True)
    hg.set_value(2, 3/4.0*pi, debug=True)
    hg.set_value(4, 3/4.0*pi, debug=True)
    hg.set_value(2, pi, debug=True)
    hg.set_value(2, 5/4.0*pi, debug=True)
    hg.set_value(2, 3/2.0*pi, debug=True)
    hg.set_value(2, 7/4.0*pi, debug=True)


def test_grid_move(hg):
    hg.move_grid(2, -1)


def test_create_histogram(hg):
    return hg.create_histogram(sector_angle=45, debug=True)


def test_draw_histogram(hg):
    hg.draw_histogram()


def test_draw_grid(hg):
    hg.draw_grid()


hg = HistogramGrid.HistogramGrid(9, 9, cell_size=1.0)
test_grid_generation(hg)
test_draw_grid(hg)
histogram = test_create_histogram(hg)
test_draw_histogram(hg)