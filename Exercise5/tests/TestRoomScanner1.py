# -*- coding: utf-8 -*-
__author__ = 'Ecki'

# Changelog:
# 14.07.2015 (Phil):    implemented obstacle avoidance with histogram grid

# Standard library imports
from math import *
# Local imports
from HTWG_Robot_Simulator_V1 import Robot as Robot, officeWorldWithDynObstacles_V1_3 as loadedWorld
from Exercise5.state_machine import StateMachine, Transitions
from Exercise5.obstacle_avoidance import HistogramGrid, PolarHistogram, Watchdog, ObstacleAvoidance
from Exercise5.util import Calculations as Calc
from Exercise5.tests import RobotLocationNoEst as RobotLocation
from Exercise5.movements import CarrotDonkey as CarrotDonkey
from Exercise5.navigation import PathScheduler
from Exercise5.localization import BoxLocator
from Exercise5.localization import RoomScanner

# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
rooms = myWorld.getRooms()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
#set_robot_opt['x'] = 2
#set_robot_opt['y'] = 9
set_robot_opt['x'] = rooms[0][1]
set_robot_opt['y'] = rooms[0][2]+3
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# define polyline
#polyline = [[6, 2], [5, 3], [5, 15], [10, 16], [13, 6], [9, 13], [9, 14], [9, 8], [1, 18]]
# polyline = [[3, 5.5], [22, 5.5]]
# polyline = [[3, 6], [9.5, 6], [11, 2]]

# Place landmarks and return their positions
myWorld.draw_landmark(9, 0)
myWorld.draw_landmark(9, 14)
landmark_positions = myWorld.get_landmark_positions()


# Set up RobotLocation
robot_loc = RobotLocation.RobotLocation(myRobot, myWorld, landmark_positions)

# Set up room scanner
room_scan = RoomScanner.RoomScanner(myWorld, robot_loc, est=False)

corners = room_scan.scan_for_corners()
for corner in corners:
     myWorld.draw_found_box(corner)

room = room_scan.get_room(robot_loc.get_robot_point(est=False))
if room is not None:
    print room.get_name()
else:
    print "None"

# Set up histogram grid for obstacle avoidance
obstacle_avoidance = ObstacleAvoidance.ObstacleAvoidance(myRobot, robot_loc, plot_grid=False)
polar_hist = PolarHistogram.PolarHistogram(myRobot, robot_loc)

# Set up CarrotDonkey
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld, robot_loc, move_backwards=False)

# Set up Watchdog
w_dog = Watchdog.Watchdog(robot_loc)

# Set up PathScheduler
path_sched = PathScheduler.PathScheduler(myWorld, skip_calculations=True)

# Set up StateMachine
transitions = Transitions.Transitions(myRobot, robot_loc, carrot_donkey, path_sched)
state_machine = StateMachine.StateMachine(transitions)

# Set up BoxLocator
box_loc = BoxLocator.BoxLocator(robot_loc, myWorld)

# calculate route
robot_point = robot_loc.get_robot_point()
path_sched.find_shortest_route(robot_point)


#################TESTING


room_scan.update()


n = int(1/0.1)
for i in xrange(n):
    movement = [0, pi]
    myRobot.move(movement)
    room_scan.update()

room_scan.update()


# close world by clicking
myWorld.close()
