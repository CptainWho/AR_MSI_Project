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
from Exercise4.util import RobotLocation
from Exercise4.localization import ParticleCloud, MCL

debug = False


# Create obstacleWorld and new Robot
myWorld = World.buildWorld()
myRobot = Robot.Robot()
robot_loc = RobotLocation.RobotLocation(myRobot)
# Place Robot in World
robot_position = [4, 7, 0]
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = robot_position[0]
set_robot_opt['y'] = robot_position[1]
set_robot_opt['theta'] = robot_position[2]
myWorld.setRobot(**set_robot_opt)

# Set up particle cloud and add particles
particle_cloud = ParticleCloud.ParticleCloud(myWorld, myRobot, draw=False)
if debug:
    particle_cloud.add_particle(9.5, 7, 0, number=0)
    particle_cloud.add_particle(9.5, 7, pi, number=1)
    particle_cloud.add_particle(8, 6, 0, number=2)
    particle_cloud.add_particle(8, 6, pi, number=3)
else:
    particle_cloud.create_particles(500, position=robot_position)

# Place landmarks and return their positions
myWorld.draw_landmark(9, 0)
myWorld.draw_landmark(9, 14)
landmark_positions = myWorld.get_landmark_positions()

print 'Landmark positions:'
if landmark_positions is not None:
    for pos in landmark_positions:
        print '\t' + str(pos)

# Set up MCL localization
mcl = MCL.MCL(particle_cloud, robot_loc=robot_loc, draw=True)

# Define movement
movement = [0.5, 0]

for j in xrange(100):
    myRobot.move(movement)
    number_dist_angles = myRobot.sense_landmarks()
    if debug:
        print number_dist_angles
        print 'Robot:'
        for i in xrange(len(number_dist_angles[0])):
            print '\tdist to landmark %d: %0.2f' % (i, number_dist_angles[1][i])
            print '\tangle to landmark %d: %0.2f' % (i, number_dist_angles[2][i] / pi * 180.0)

    # Run MCL
    est_x, est_y, est_theta = mcl.mcl_landmark(movement, landmark_positions, sensor_data=number_dist_angles)

    # real_x, real_y, real_theta = robot_loc.get_robot_position()
    #
    # print 'Real robot location:'
    # print '\tx=%0.2f y=%0.2f theta=%0.2f' % (real_x, real_y, real_theta)
    # print ' Estimated robot location:'
    # print '\tx=%0.2f y=%0.2f theta=%0.2f' % (est_x, est_y, est_theta)

myWorld.close()