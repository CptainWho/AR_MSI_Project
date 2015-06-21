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
# Local imports
from HTWG_Robot_Simulator_V1 import Robot, officeWorld as World
from Exercise4.localization import ParticleCloud


# Create obstacleWorld and new Robot
myWorld = World.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 2
set_robot_opt['y'] = 2
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# Set up particle cloud
particle_cloud = ParticleCloud.ParticleCloud(myWorld)
particle_cloud.create_particles(10)
# particle = ParticleCloud.Particle(0, 3, 4, 0, myWorld)
# particle_cloud.append(particle)

# Place landmarks
myWorld.draw_landmark(3, 2)
myWorld.draw_landmark(2, 4)

landmark_positions = myWorld.get_landmark_positions()

print 'Landmark positions:'
if landmark_positions is not None:
    for pos in landmark_positions:
        print '\t' + str(pos)

for j in xrange(1):
    myRobot.move([0, -pi])
    number_dist_angles = myRobot.sense_landmarks()
    print number_dist_angles
    print 'Robot:'
    for i in xrange(len(number_dist_angles[0])):
        print '\tdist to landmark %d: %0.2f' % (i, number_dist_angles[1][i])
        print '\tangle to landmark %d: %0.2f' % (i, number_dist_angles[2][i])

    for particle in particle_cloud:
        weight = particle.calculate_weight(landmark_positions, number_dist_angles[1], number_dist_angles[2], debug=True)

# Change color of particles and increment their numbers by 1
for particle in particle_cloud:
    p_x, p_y = particle.get_pos()
    p_number = particle.get_number()
    particle(p_x, p_y, color='red', number=p_number)

# particle(10, 2, color='red', number=0)

myWorld.close()