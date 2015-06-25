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
set_robot_opt['x'] = 9
set_robot_opt['y'] = 7
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# Set up particle cloud
particle_cloud = ParticleCloud.ParticleCloud(myWorld, myRobot, draw=True)
# particle_cloud.create_particles(10)
particle_cloud.add_particle(9.5, 7, 0, number=0)
particle_cloud.add_particle(9.5, 7, pi, number=1)
particle_cloud.add_particle(8, 6, 0, number=2)
particle_cloud.add_particle(8, 6, pi, number=3)

# Place landmarks
myWorld.draw_landmark(10, 7)
myWorld.draw_landmark(9, 9)

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
        print '\tangle to landmark %d: %0.2f' % (i, number_dist_angles[2][i] / pi * 180.0)

    for particle in particle_cloud:
        weight = particle.calculate_weight(landmark_positions, number_dist_angles[1], number_dist_angles[2], debug=True)

# Change color of particles and increment their numbers by 1
for particle in particle_cloud:
    p_x, p_y = particle.get_pos()
    p_number = particle.get_number()
    p_theta = particle.get_theta()
    particle(p_x, p_y, p_theta, color='red', number=p_number)

# particle(10, 2, color='red', number=0)

myWorld.close()