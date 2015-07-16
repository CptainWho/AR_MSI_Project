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
#set_robot_opt['y'] = 6
set_robot_opt['x'] = rooms[0][1]
set_robot_opt['y'] = rooms[0][2]
set_robot_opt['theta'] = pi/2
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

room_scan = RoomScanner.RoomScanner(myWorld, robot_loc, est=False)


# Set up PathScheduler
path_sched = PathScheduler.PathScheduler(myWorld, skip_calculations=True)
path_sched.find_shortest_route(robot_loc.get_robot_point())

# Set up histogram grid for obstacle avoidance
obstacle_avoidance = ObstacleAvoidance.ObstacleAvoidance(myRobot, robot_loc, plot_grid=False)
polar_hist = PolarHistogram.PolarHistogram(myRobot, robot_loc)

# Set up CarrotDonkey
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld, robot_loc, move_backwards=False)

# Set up Watchdog
w_dog = Watchdog.Watchdog(robot_loc)


# Set up StateMachine
transitions = Transitions.Transitions(myRobot, robot_loc, carrot_donkey, path_sched)
state_machine = StateMachine.StateMachine(transitions)

# Set up BoxLocator
box_loc = BoxLocator.BoxLocator(robot_loc, myWorld)

# calculate route
robot_point = robot_loc.get_robot_point()
#path_sched.find_shortest_route(robot_point)
polyline = path_sched.get_next_polyline()
myWorld.drawPolyline(polyline)


carrot_donkey.set_polyline(polyline, 0.8)

target_reached = False
states = {'NoObstacle', 'Obstacle', 'RoomReached', 'Finished'}
[v, omega] = [0, 0]
movement_old = [0, 0]

while not target_reached:
    next_point = carrot_donkey.get_next_point()
    state = state_machine.next_state()
    if state in states:
        if state == 'NoObstacle':
            [v, omega] = carrot_donkey.next_movement_commands()

        if state == 'Obstacle':
            carrot_pos = carrot_donkey.carrot.get_pos()
            # movement = obstacle_avoidance.avoid_obstacle(carrot_pos)
            # if movement is None:
            #     # Use old v and omega
            #     [v, omega] = movement_old
            # else:
            #     [v, omega] = movement
            [v, omega] = polar_hist.avoid_obstacle(carrot_pos)
            carrot_donkey.place_carrot_above_robot()

            #[v, omega] = histogram.avoid_obstacle(robot_loc, carrot_donkey.get_next_point())

        if state == 'RoomReached':
            polyline = path_sched.get_next_polyline()
            polyline = Calc.douglas_peucker(polyline, 0.2)
            myWorld.drawPolyline(polyline)
            carrot_donkey.set_polyline(polyline, 0.8)

        if state == 'Finished':
            target_reached = True

        movement = w_dog.move_robot(myRobot, [v, omega])
        movement_old = movement
        # Update estimated robot position
        #robot_loc.update_robot_position_est(movement)
        box_loc.update_boxes()
        room_scan.update()

box_loc.print_found_boxes()
box_loc.draw_found_boxes()

# close world by clicking
myWorld.close()
