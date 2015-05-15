# -*- coding: utf-8 -*-
__author__ = 'Ecki'

"""
Erweitern Sie nun Ihren Linienverfolger followPolyline() um eine Hindernisvermeidung. Ein
vorgegebener Polygonzug soll möglichst genau abgefahren werden, wobei Hindernissen
ausgewichen werden muss. Die folgenden Abbildungen zeigen für vorgegebene Polygonzüge
(grün) typische Trajektorien (rot).
Hinweis: Hilfreich könnte sein, den Linenverfolger als endlichen Automaten mit folgenden
Zuständen zu realisieren:
-   der Roboter fährt auf den nächsten Eckpunkt zu und hat kein Hindernis in unmittelbarer
    Nähe in Fahrtrichtung.
-   der Roboter fährt auf den nächsten Eckpunkt zu, sieht aber ein Hindernis in Fahrtrichtung:
    er wählt aufgrund der Sensordaten eine neue Richtung, die möglichst nahe an der
    Zielrichtung liegt, aber frei befahrbar ist (siehe auch Histogrammverfahren).
-   der Roboter hat einen Eckpunkt erreicht und richtet sich auf den nächsten Eckpunkt aus.
Die Geschwindigkeit sollte aus Sicherheitsgründen umgekehrt proportional zur Rotationsgeschwindigkeit
gewählt werden.
"""

from math import *
from HTWG_Robot_Simulator_V1 import obstacleWorld
from HTWG_Robot_Simulator_V1 import officeWorld
from HTWG_Robot_Simulator_V1 import Robot
import A2_StateMachine as StateMachine

from datetime import datetime, timedelta

# Roboter in einer Welt positionieren:
myWorld = obstacleWorld.buildWorld()
#myWorld = officeWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 4.5, 6, pi/2)

def obstacleInSight():
    sensorDist = myRobot.sense()
    sensorDirections = myRobot.getSensorDirections()
    result = False
    return result

""" States """
class NoObstacle(StateMachine.State):
    def run(self):
        print "Running with no Obstacle"

    def next(self, input):
        if input == True:


""" Static variable init """
myStateM.noobstacle = NoObstacle()


""" Main """

myStateM = StateMachine.A2_StateMachine(StateMachine.A2_StateMachine.NoObstacle)
myStateM.applyStates(obstacleInSight())





# close world by clicking
myWorld.close()