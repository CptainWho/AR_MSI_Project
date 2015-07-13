# -*- coding: utf-8 -*-
__author__ = 'Ecki'

# Standard library imports
from math import *
# Local imports
from HTWG_Robot_Simulator_V1 import Robot as Robot, officeWorldWithDynObstacles_V1_3 as loadedWorld
from Exercise5.state_machine import StateMachine, Transitions
from Exercise5.obstacle_avoidance import HistogramGrid
from Exercise5.obstacle_avoidance import PolarHistogram
from Exercise5.util import RobotLocation
from Exercise5.movements import CarrotDonkey as CarrotDonkey
from Exercise5.obstacle_avoidance import Watchdog
from Exercise5.navigation import PathScheduler
from Exercise5.util import Calculations as Calc

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



# Set up StateMachine
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld)

robot_loc = RobotLocation.RobotLocation(myRobot)
#histogram = HistogramGrid.HistogramGrid(5, 5, cell_size=0.1, hist_threshold=5.0)
polar_hist = PolarHistogram.PolarHistogram(myRobot, robot_loc)
w_dog = Watchdog.Watchdog(robot_loc)
path_sched = PathScheduler.PathScheduler(myWorld)
transitions = Transitions.Transitions(myRobot, carrot_donkey, path_sched)
state_machine = StateMachine.StateMachine(transitions)

polyline = path_sched.find_nearest_room(robot_loc.get_robot_point())
myWorld.drawPolyline(polyline)


carrot_donkey.set_polyline(polyline, 0.8)

target_reached = False
states = {'NoObstacle', 'Obstacle', 'RoomReached'}
[v, omega] = [0, 0]

while not target_reached:
    next_point = carrot_donkey.get_next_point()
    state = state_machine.next_state()
    if state in states:
        if state == 'NoObstacle':
            [v, omega] = carrot_donkey.next_movement_commands()

        if state == 'Obstacle':
            carrot_pos = carrot_donkey.carrot.get_pos()
            [v, omega] = polar_hist.avoid_obstacle(carrot_pos)
            #[v, omega] = polar_hist.avoid_obstacle(next_point)
            carrot_donkey.place_carrot_above_robot()

            #[v, omega] = histogram.avoid_obstacle(robot_loc, carrot_donkey.get_next_point())

        if state == 'RoomReached':
            polyline = path_sched.find_nearest_room(robot_loc.get_robot_point())
            polyline = Calc.douglas_peucker(polyline, 0.2)
            myWorld.drawPolyline(polyline)
            carrot_donkey.set_polyline(polyline, 0.8)

        w_dog.move_robot(myRobot, [v, omega])


# close world by clicking
myWorld.close()
