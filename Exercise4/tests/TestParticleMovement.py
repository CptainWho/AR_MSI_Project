# -*- coding: utf-8 -*-
""" Module TestLandmark
"""

__project__ = 'Exercise 4'
__module__  = 'TestLandmark'
__author__  = 'Philipp Lohrer'
__date__    = '21.06.2015'

__version__ = '0.1'

# Standard library imports
import numpy as np
from math import pi
from time import sleep
# Local imports
from HTWG_Robot_Simulator_V1 import Robot, officeWorld as World
from Exercise4.localization import ParticleCloud


# Create obstacleWorld and new Robot
myWorld = World.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 9
set_robot_opt['y'] = 7
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# Set up particle cloud
particle_cloud = ParticleCloud.ParticleCloud(myWorld, myRobot, draw=True)
particle_cloud.create_particles(10)
particle_cloud.add_particle(9, 8, 0)
movement = [0.5, 0]

for j in xrange(20):
    # print 'Perform movement v=%0.2f, omega=%0.2f' % (movement[0], movement[1])
    myRobot.move(movement)
    particle_cloud.move_particles(movement)
    # print 'sleep a bit...'
    # sleep(5)

myWorld.close()