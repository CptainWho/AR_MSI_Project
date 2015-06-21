# -*- coding: utf-8 -*-
""" Module TestBrushfire
"""

__project__ = 'Exercise 3'
__module__  = 'TestBrushfire'
__author__  = 'Philipp Lohrer'
__date__    = '16.06.2015'

__version__ = '1.0'

# Standard library imports
import numpy as np
import time
from math import pi
import matplotlib.pyplot as plt
# Local imports
from Exercise3.navigation import Brushfire
from HTWG_Robot_Simulator_V1 import OccupancyGrid, Robot,  officeWorld as World

# Create obstacleWorld and new Robot
myWorld = World.buildWorld()
myRobot = Robot.Robot()

occupancy_grid = myWorld.getOccupancyGrid()

# occupancy_grid = OccupancyGrid.OccupancyGrid(0,0,8,8,0.1)
# occupancy_grid.addLine(0, 0, 8, 0)
# occupancy_grid.addLine(0, 8, 8, 8)
# occupancy_grid.addLine(0, 0, 0, 8)
# occupancy_grid.addLine(8, 0, 8, 8)
# occupancy_grid.addLine(0, 4, 4, 4)
# occupancy_grid.setValue(2, 2)
# occupancy_grid.setValue(6, 6)

# occupancy_grid = OccupancyGrid.OccupancyGrid(0,0,2,2,1)
# occupancy_grid.addLine(0, 0, 2, 0)  # bottom
# occupancy_grid.addLine(0, 2, 2, 2)  # top
# occupancy_grid.addLine(0, 0, 0, 2)  # left
# occupancy_grid.addLine(2, 0, 2, 2)  # right
# occupancy_grid.setValue(2, 2)

# occupancy_grid.drawGrid()
# occupancy_grid.printGrid()

brushfire = Brushfire.Brushfire(occupancy_grid, myRobot)

brushfire_grid = brushfire.apply_brushfire(adjacency=8, safety_distance=0.5)

occupancy_grid.drawGrid(dynamic=0)

# plot = plt.imshow(brushfire_grid.transpose(), origin='lower')
# cbar = plt.colorbar(mappable=plot)
# cbar.set_label('Occupancy')
# plt.show()

