# -*- coding: utf-8 -*-
""" Module main
"""

__project__ = 'Project'
__module__  = 'main'
__author1__  = 'Philipp Lohrer'
__author2__  = 'Daniel Eckstein'
__date__    = '21.07.2015'

__version__ = '1.0'

# Standard library imports
from math import *
# Local imports
from Project.HTWG_Robot_Simulator import Robot as Robot, officeWorldWithDynObstacles_V1_3 as loadedWorld
from Project.state_machine import StateMachine, Transitions
from Project.obstacle_avoidance import Watchdog, ObstacleAvoidance
from Project.util import RobotLocation, Calculations as Calc
from Project.movements import CarrotDonkey as CarrotDonkey
from Project.navigation import PathScheduler
from Project.localization import BoxLocator, RoomScanner
from copy import deepcopy

# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
rooms = myWorld.getRooms()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
# robot_start_point = [rooms[0][1], rooms[0][2]]
start_room = rooms[0]
robot_start_point = [start_room[1], start_room[2]]
set_robot_opt['x'] = robot_start_point[0]
set_robot_opt['y'] = robot_start_point[1]
set_robot_opt['theta'] = pi/2
myWorld.setRobot(**set_robot_opt)

# Place landmarks and return their positions
myWorld.draw_landmark(9, 0.5)
myWorld.draw_landmark(9, 13.5)
landmark_positions = myWorld.get_landmark_positions()

new_world = loadedWorld.buildWorld()
grid_for_path_sched = deepcopy(new_world.getOccupancyGrid(0.5))
new_world.close(False)

# Set up RobotLocation
loc_amount_particles = 500
loc_draw_mode = 'estimation'  # localization draw mode: estimation, particle, particle_number, particle_estimation
loc_plot_errors = False
robot_loc = RobotLocation.RobotLocation(myRobot, myWorld, landmark_positions, loc_amount_particles,
                                        draw=loc_draw_mode,  plot_errors=loc_plot_errors)

# Set up RoomScanner
room_scan = RoomScanner.RoomScanner(myWorld, robot_loc, est=True)

# Set up PathScheduler
path_sched = PathScheduler.PathScheduler(myWorld, grid_for_path_sched, start_room=start_room, skip_calculations=False)
path_sched.find_shortest_route(robot_start_point)

# Set up histogram grid for obstacle avoidance
hist_plot_grid = False
obstacle_avoidance = ObstacleAvoidance.ObstacleAvoidance(myRobot, robot_loc, plot_grid=hist_plot_grid)

# Set up CarrotDonkey
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld, robot_loc, move_backwards=False)

# Set up Watchdog
w_dog = Watchdog.Watchdog(robot_loc)

# Set up StateMachine
transitions = Transitions.Transitions(myRobot, robot_loc, carrot_donkey, path_sched, room_scan)
state_machine = StateMachine.StateMachine(transitions)

# Set up BoxLocator
box_loc = BoxLocator.BoxLocator(robot_loc, myWorld)

polyline = path_sched.get_next_polyline()
myWorld.drawPolyline(polyline)

carrot_speed = 1.0
carrot_donkey.set_polyline(polyline, carrot_speed)

target_reached = False
states = {'NoObstacle', 'Obstacle', 'RoomReached', 'InspectCorners', 'RotateToExit', 'Finished'}
[v, omega] = [0, 0]
movement_old = [0, 0]
v_total = 0
time_steps = 0

while not target_reached:
    next_point = carrot_donkey.get_next_point()
    state = state_machine.next_state()
    if state in states:
        if state == 'NoObstacle':
            [v, omega] = carrot_donkey.next_movement_commands()

        if state == 'Obstacle':
            carrot_pos = carrot_donkey.carrot.get_pos()
            movement = obstacle_avoidance.avoid_obstacle(carrot_pos)
            if movement is None:
                # Use old v and omega
                [v, omega] = movement_old
            else:
                [v, omega] = movement
            carrot_donkey.place_carrot_above_robot()

        if state == 'RoomReached':
            polyline = path_sched.get_next_polyline()
            polyline = Calc.douglas_peucker(polyline, 0.2)
            myWorld.drawPolyline(polyline)
            carrot_donkey.set_polyline(polyline, carrot_speed)

        if state == 'InspectCorners':
            # rotate to all available corners
            open_list = room_scan.get_open_list()
            if len(open_list) > 0 and open_list is not None:
                corner = open_list[0]
                [v, omega] = carrot_donkey.rotate_to_point(corner)

        if state == 'RotateToExit':
            # place carrot in new direction
            carrot_donkey.place_carrot_above_robot()
            # rotate to carrot
            [v, omega] = carrot_donkey.rotate_to_carrot()

        if state == 'Finished':
            target_reached = True

        movement = w_dog.move_robot(myRobot, [v, omega])
        movement_old = movement
        v_total += movement[0]
        time_steps += 1
        # Update estimated robot position
        robot_loc.update_robot_position_est(movement)
        room_scan.update()
        box_loc.update_boxes(room_scan.get_room(robot_loc.get_robot_point()))

print '########################################'
print 'Mean Robot speed:'
print '\t%0.2f' % (v_total / time_steps)

box_loc.print_found_boxes()
box_loc.draw_found_boxes()

robot_loc.print_localization_errors()

# close world by clicking
myWorld.close()
