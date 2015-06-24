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
from HTWG_Robot_Simulator_V1 import Robot as Robot, emptyWorld as loadedWorld
from Exercise3.navigation import AStarAlgo as AStarAlgo, Brushfire
from Exercise3.util import RobotLocation
from Exercise3.movements import CarrotDonkey
from Exercise3.util import Calculations as Calc


# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()


polyline = [[5.0, 9.0], [5.0, 9.125], [5.0, 9.25], [5.0, 9.375], [5.0, 9.5], [5.0, 9.625], [5.0, 9.75], [5.0, 9.875], [5.0, 10.0], [5.0, 10.125], [5.0, 10.25], [5.0, 10.375], [5.0, 10.5], [5.0, 10.625], [5.0, 10.75], [5.0, 10.875], [5.0, 11.0], [5.0, 11.125], [5.0, 11.25], [5.0, 11.375], [4.875, 11.5], [4.75, 11.625], [4.625, 11.75], [4.5, 11.875], [4.375, 11.875], [4.25, 12.0], [4.125, 12.0], [4.0, 12.0], [3.875, 12.0], [3.75, 12.0], [3.625, 12.0], [3.5, 12.0], [3.375, 12.0], [3.25, 12.0], [3.125, 12.0], [3.0, 12.0], [2.875, 11.875], [2.75, 11.75], [2.625, 11.625], [2.5, 11.5], [2.375, 11.375], [2.25, 11.25], [2.125, 11.125], [2.0, 11.0], [2.0, 10.875], [2.0, 10.75], [2.0, 10.625], [2.0, 10.5], [2.0, 10.375], [2.0, 10.25], [2.0, 10.125], [2.0, 10.0], [2.0, 9.875], [2.0, 9.75], [2.0, 9.625], [2.0, 9.5], [2.0, 9.375], [2.0, 9.25], [2.0, 9.125], [2.0, 9.0], [2.0, 8.875], [2.0, 8.75], [2.0, 8.625], [2.0, 8.5], [2.0, 8.375], [2.0, 8.25], [2.0, 8.125], [2.0, 8.0], [2.125, 7.875], [2.25, 7.75], [2.375, 7.625], [2.5, 7.5], [2.625, 7.375], [2.75, 7.25], [2.875, 7.125], [3.0, 7.125], [3.125, 7.0], [3.25, 7.0], [3.375, 7.0], [3.5, 7.0], [3.625, 7.0], [3.75, 7.0], [3.875, 7.0], [4.0, 7.0], [4.125, 6.875], [4.25, 6.75], [4.375, 6.625], [4.5, 6.5], [4.625, 6.375], [4.75, 6.25], [4.875, 6.125], [5.0, 6.0], [5.0, 5.875], [5.0, 5.75], [5.0, 5.625], [5.0, 5.5], [5.125, 5.375], [5.25, 5.375], [5.375, 5.25], [5.5, 5.25], [5.625, 5.125], [5.75, 5.125], [5.875, 5.125], [6.0, 5.0], [6.125, 4.875], [6.25, 4.75], [6.375, 4.625], [6.5, 4.5], [6.625, 4.5], [6.75, 4.5], [6.875, 4.5], [7.0, 4.625], [7.125, 4.5], [7.25, 4.5], [7.375, 4.5], [7.5, 4.5], [7.625, 4.5], [7.75, 4.5], [7.875, 4.5], [8.0, 4.5], [8.125, 4.5], [8.25, 4.5], [8.375, 4.5], [8.5, 4.5], [8.625, 4.5], [8.75, 4.5], [8.875, 4.5], [9.0, 4.5], [9.125, 4.375], [9.25, 4.375], [9.375, 4.25], [9.5, 4.25], [9.625, 4.125], [9.75, 4.125], [9.875, 4.0]]

index = 4
the_line = polyline[index:len(polyline)]

myWorld.drawPolyline(polyline)

polyline = Calc.douglas_peucker(polyline, 0.5)

myWorld.drawPolyline(polyline)



myWorld.close()