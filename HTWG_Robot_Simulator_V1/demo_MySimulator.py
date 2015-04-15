""" Module 
Module Description:
"""

__project__ = ''
__module__  = ''
__author__  = 'Philipp Lohrer'
__email__   = 'plohrer@htwg-konstanz.de'
__date__    = '02.04.2015'

__version__ = '0.1'

#################################################################
## Changelog:
#
#################################################################
## Imports
#################################################################
# Standard library imports
from math import *
# Local imports
import emptyWorld
import Robot
import World

#################################################################

def demo_Simulator_1():
    # Anzahl Zeitschritte n mit jeweils der Laenge T = 0.1 sec definieren.
    # T laesst sich ueber die Methode myRobot.setTimeStep(T) einstellen.
    # T = 0.1 sec ist voreingestellt.
    n = 150

    # Definiere Folge von Bewegungsbefehle:
    motionCircle = [[1, -24 * pi / 180] for i in range(n)]

    # Bewege Roboter
    for t in range(n):
        # Bewege Roboter
        motion = motionCircle[t]
        print "v = ", motion[0], "omega = ", motion[1]*180/pi
        myRobot.move(motion)

        # Gib Daten vom Distanzsensor aus:
        distanceSensorData = myRobot.sense()
        print "Dist Sensor: ", distanceSensorData

        # Gib Daten vom Boxsensor aus:
        boxSensorData = myRobot.senseBoxes()
        if boxSensorData != None:
            print "Box Sensor: ", boxSensorData
            

## Create empty World and new Robot
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
## Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 2
set_robot_opt['y'] = 5.5
set_robot_opt['theta'] = pi/2
myWorld.setRobot(**set_robot_opt)

demo_Simulator_1()

## Close Simulation
myWorld.close()














