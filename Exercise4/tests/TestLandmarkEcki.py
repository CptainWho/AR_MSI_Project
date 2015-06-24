# -*- coding: utf-8 -*-

__author__  = 'Ecki'

# Standard library imports
import numpy as np
from math import pi
# Local imports
from HTWG_Robot_Simulator_V1 import Robot, officeWorld as World
from Exercise4.localization import ParticleCloud


# Create obstacleWorld and new Robot
myWorld = World.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 9.5
set_robot_opt['y'] = 7
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# Set up particle cloud
particle_cloud = ParticleCloud.ParticleCloud(myWorld)
particle_cloud.create_particles(100)
# particle0 = ParticleCloud.Particle(0, 9.5, 7, 0, myWorld)
# particle1 = ParticleCloud.Particle(1, 9.5, 7, pi, myWorld)
# particle2 = ParticleCloud.Particle(2, 8, 6, 0, myWorld)
# particle3 = ParticleCloud.Particle(3, 8, 6, pi, myWorld)
# particle_cloud.append([particle0, particle1, particle2, particle3])

# Place landmarks
myWorld.draw_landmark(9.5, 1, number=1)
myWorld.draw_landmark(9.5, 13, number=2)

lmrks = myRobot.sense_landmarks()
print lmrks

myWorld.close()