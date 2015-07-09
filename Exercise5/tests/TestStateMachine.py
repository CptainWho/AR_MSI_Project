# -*- coding: utf-8 -*-
__author__ = 'Ecki'

# Standard library imports
from math import *
# Local imports
from HTWG_Robot_Simulator_V1 import Robot as Robot, obstacleWorld1 as loadedWorld
from Exercise5.state_machine import StateMachine, Transitions
from Exercise5.obstacle_avoidance import PolarHistogram
from Exercise5.util import RobotLocation
from Exercise5.movements import CarrotDonkey


# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
#set_robot_opt['x'] = 2
#set_robot_opt['y'] = 6
set_robot_opt['x'] = 7
set_robot_opt['y'] = 1
set_robot_opt['theta'] = pi/2
myWorld.setRobot(**set_robot_opt)

# define polyline
polyline = [[6, 2], [5, 3], [5, 15], [10, 16], [13, 6], [9, 13], [9, 14], [9, 8], [1, 18]]
# polyline = [[3, 5.5], [22, 5.5]]
# polyline = [[3, 6], [9.5, 6], [11, 2]]
myWorld.drawPolyline(polyline)


# Set up StateMachine
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld)
transitions = Transitions.Transitions(myRobot, carrot_donkey)
state_machine = StateMachine.StateMachine(transitions)
robot_loc = RobotLocation.RobotLocation(myRobot)
polar_hist = PolarHistogram.PolarHistogram(myRobot, robot_loc)


carrot_donkey.set_polyline(polyline, 0.8)

target_reached = False
states = {'NoObstacle', 'Obstacle'}
[v, omega] = [0, 0]

while not target_reached:
    next_point = carrot_donkey.get_next_point()
    state = state_machine.next_state()
    if state in states:
        if state == 'NoObstacle':
            [v, omega] = carrot_donkey.next_movement_commands()

        if state == 'Obstacle':
            [v, omega] = polar_hist.avoid_obstacle(next_point)

        myRobot.move([v, omega])


# close world by clicking
myWorld.close()
