# -*- coding: utf-8 -*-
""" Module TestBrushfire
"""

__project__ = 'Exercise 3'
__module__  = 'TestBrushfire'
__author__  = 'Philipp Lohrer'
__date__    = '11.06.2015'

__version__ = '0.1'

# Standard library imports
import numpy as np
from math import pi
# Local imports
from Exercise3.robot_navigation import Brushfire
from HTWG_Robot_Simulator_V1 import OccupancyGrid, Robot,  officeWorld as World

# Create obstacleWorld and new Robot
myWorld = World.buildWorld()
myRobot = Robot.Robot()

occupancy_grid = myWorld.getOccupancyGrid()
occupancy_grid.drawGrid()
# occupancy_grid.printGrid()

brushfire_grid = Brushfire.apply_brushfire(occupancy_grid, myRobot, safety_distance=None)
brushfire_grid.drawGrid()

