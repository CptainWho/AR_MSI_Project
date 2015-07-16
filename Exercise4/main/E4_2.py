# -*- coding: utf-8 -*-
""" Module E4
"""

__project__ = 'Exercise 4'
__module__  = 'E4'
__author1__  = 'Philipp Lohrer'
__author1__  = 'Daniel Eckstein'
__date__    = '14.07.2015'

__version__ = '1.0'

# Standard library imports
import numpy as np
from math import pi
# Local imports
from HTWG_Robot_Simulator_V1 import Robot, emptyWorld as World
from Exercise5.util import RobotLocation
from Exercise4.localization import ParticleCloud, MCL

# Create obstacleWorld and new Robot
myWorld = World.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
robot_position = [10, 5, 0]  # [15, 5, 0]  # [4, 7, 0]
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = robot_position[0]
set_robot_opt['y'] = robot_position[1]
set_robot_opt['theta'] = robot_position[2]
myWorld.setRobot(**set_robot_opt)

# Place landmarks and return their positions
landmarks = 2
if landmarks == 2:
    myWorld.draw_landmark(9, 0)
    myWorld.draw_landmark(9, 14)
elif landmarks == 4:
    myWorld.draw_landmark(0, 0)
    myWorld.draw_landmark(19, 0)
    myWorld.draw_landmark(0, 14)
    myWorld.draw_landmark(19, 14)
elif landmarks == 6:
    myWorld.draw_landmark(0, 0)
    myWorld.draw_landmark(9, 0)
    myWorld.draw_landmark(19, 0)
    myWorld.draw_landmark(0, 14)
    myWorld.draw_landmark(9, 14)
    myWorld.draw_landmark(19, 14)
landmark_positions = myWorld.get_landmark_positions()

# Create instances
robot_loc = RobotLocation.RobotLocation(myRobot, myWorld, landmark_positions)

# Define start and end points
start_point = robot_loc.get_robot_point()

######################################################################################

# Set up particle cloud and add particles
particle_cloud = ParticleCloud.ParticleCloud(myWorld, myRobot, draw='estimation')
particle_cloud.create_particles(500, position=robot_position)

# Set up MCL localization
mcl = MCL.MCL(particle_cloud, robot_loc=robot_loc, draw=True)

print 'Landmark positions:'
if landmark_positions is not None:
    for pos in landmark_positions:
        print '\t' + str(pos)

for x in xrange(500):
    movement = [0.5, pi/16.0]
    myRobot.move(movement)
    number_dist_angles = myRobot.sense_landmarks()
    # Run MCL
    robot_position = mcl.mcl_landmark(movement, landmark_positions, sensor_data=number_dist_angles, debug=False)

myWorld.close()
