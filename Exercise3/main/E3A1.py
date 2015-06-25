# -*- coding: utf-8 -*-
""" Module Aufgabe 1
Module Description:
Der Robotersimulator bietet die Möglichkeit an, aus der definierten Umgebungskarte ein
Belegheitsgitter zu generieren. Die Gittergröße kann frei gewählt werden.
Implementierten Sie ein A*-Verfahren, das mit Hilfe des Belegheitsgitters einen möglichst kurzen
Weg zu einem Zielpunkt p als Polygonzug berechnet.
"""

__project__ = 'Exercise 3'
__module__ = 'E3_A1'
__author1__ = 'Ecki'
__author2__ = 'Philipp Lohrer'
__date__ = '19.06.2015'

__version__ = '1.0'

from datetime import datetime
from HTWG_Robot_Simulator_V1 import Robot as Robot, officeWorld as loadedWorld
from Exercise3.navigation import AStarAlgo as AStarAlgo, Brushfire
from Exercise3.util import RobotLocation
from Exercise3.movements import CarrotDonkey
from Exercise3.util import Calculations as Calc


# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 5
set_robot_opt['y'] = 9
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# create instances
robot_loc = RobotLocation.RobotLocation(myRobot)
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld)
occupancy_grid = myWorld.getOccupancyGrid(0.25)
brushfire = Brushfire.Brushfire(occupancy_grid, myRobot)
t_start = datetime.now()
brushfire.apply_brushfire(adjacency=8, safety_distance=0.25)
t_delta = (datetime.now() - t_start).total_seconds()
print 'Brushfire in: %0.5f seconds' % t_delta
a_star = AStarAlgo.AStarAlgorithm(occupancy_grid)

# Draw occupancy grid with applied brushfire
occupancy_grid.drawGrid()

# define points
start_point = robot_loc.get_robot_point()
end_point = [10, 4]
myWorld.addBox(end_point[0], end_point[1])

# Find shortest path from start_point to end_point
t_start = datetime.now()
polyline = a_star.dijkstra_algorithm(start_point, end_point, adjacency=8)
t_delta = (datetime.now() - t_start).total_seconds()
print 'Found shortest path in: %0.5f seconds' % t_delta

print "Shortest path found! --> Draw polyline"
myWorld.drawPolyline(polyline)

# apply douglas peucker algorithm
polyline = Calc.douglas_peucker(polyline, 0.2)
myWorld.drawPolyline(polyline)

# Follow polyline via carrot-donkey
carrot_donkey.setCarrotPosition(polyline[0])
for p_next in polyline:
    while carrot_donkey.carrot_pos != p_next:
        carrot_donkey.moveCarrotToPoint(p_next, 0.5)
        myRobot.move(carrot_donkey.followCarrot())

myWorld.close()