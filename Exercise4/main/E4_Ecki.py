# -*- coding: utf-8 -*-
""" Module E4
"""

__project__ = 'Exercise 4'
__module__  = 'E4'
__author1__  = 'Philipp Lohrer'
__author1__  = 'Daniel Eckstein'
__date__    = '06.07.2015'

__version__ = '1.0'

# Standard library imports
import numpy as np
from math import pi
# Local imports
from HTWG_Robot_Simulator_V1 import Robot, emptyWorld as World
from Exercise4.util import RobotLocation, Calculations as Calc
from Exercise4.localization import ParticleCloud, MCL
from Exercise4.navigation import AStarAlgo as AStarAlgo, Brushfire
from Exercise4.movements import CarrotDonkey

# Create obstacleWorld and new Robot
myWorld = World.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
robot_position = [15, 5, 0]  # [4, 7, 0]
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = robot_position[0]
set_robot_opt['y'] = robot_position[1]
set_robot_opt['theta'] = robot_position[2]
myWorld.setRobot(**set_robot_opt)

# Create instances
robot_loc = RobotLocation.RobotLocation(myRobot)
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld)
occupancy_grid = myWorld.getOccupancyGrid(0.25)
brushfire = Brushfire.Brushfire(occupancy_grid, myRobot)

# Apply brushfire to occupancy_grid and initialize A-Star-Algorithm
brushfire.apply_brushfire(adjacency=4, safety_distance=0.25)
a_star = AStarAlgo.AStarAlgorithm(occupancy_grid)
# Draw occupancy grid with applied brushfire
# occupancy_grid.drawGrid()

# Define start and end points
start_point = robot_loc.get_robot_point()
end_point = [4, 7]  # [15, 5]
myWorld.addBox(end_point[0], end_point[1])

# Find shortest path from start_point to end_point
polyline = a_star.dijkstra_algorithm(start_point, end_point, adjacency=8)
# Apply douglas peucker algorithm and draw polyline in world
polyline = Calc.douglas_peucker(polyline, 0.2)
myWorld.drawPolyline(polyline)

######################################################################################

# Set up particle cloud and add particles
particle_cloud = ParticleCloud.ParticleCloud(myWorld, myRobot, draw='particle_number')
particle_cloud.create_particles(100, position=robot_position)

# Set up MCL localization
mcl = MCL.MCL(particle_cloud, robot_loc=robot_loc, draw=False)

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

print 'Landmark positions:'
if landmark_positions is not None:
    for pos in landmark_positions:
        print '\t' + str(pos)

# Follow polyline via carrot-donkey
carrot_donkey.setCarrotPosition(polyline[0])
for p_next in polyline:
    # while not Calc.point_in_tol(robot_position[0:2], end_point, tol=0.2):
    while carrot_donkey.carrot_pos != p_next:
        carrot_donkey.moveCarrotToPoint(p_next, 0.5)
        movement = carrot_donkey.followCarrot(robot_position)
        myRobot.move(movement)
        number_dist_angles = myRobot.sense_landmarks()
        # Run MCL
        robot_position = mcl.mcl_landmark(movement, landmark_positions, sensor_data=number_dist_angles, debug=False)

myWorld.close()
