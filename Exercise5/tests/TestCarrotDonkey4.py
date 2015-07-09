# -*- coding: utf-8 -*-
__author__ = 'Ecki'

# Standard library imports
from math import *
# Local imports
from HTWG_Robot_Simulator_V1 import Robot as Robot, obstacleWorld1 as loadedWorld
from Exercise2.util import StateMachine, Transitions
from Exercise2.movements import BasicMovement
from Exercise2.obstacle_avoidance import PolarHistogram
from Exercise2.util import Polyline
from Exercise2.util import RobotLocation


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

polyline = Polyline.Polyline(polyline)

# Set up StateMachine
transitions = Transitions.Transitions(myRobot, polyline)
state_machine = StateMachine.StateMachine(transitions)
basic_mov = BasicMovement.BasicMovement(myRobot)
robot_loc = RobotLocation.RobotLocation(myRobot)
polar_hist = PolarHistogram.PolarHistogram(myRobot, robot_loc)



target_reached = False
states = {'NoObstacle', 'Obstacle', 'CornerReached', 'TargetReached'}
[v, omega] = [0, 0]

while not target_reached:
    next_point = polyline.get_next_point()
    state = state_machine.next_state()
    if state in states:
        if state == 'NoObstacle':
            [v, omega] = basic_mov.follow_line(polyline.get_last_point(), next_point, 0.5)

        if state == 'Obstacle':
            [v, omega] = polar_hist.avoid_obstacle(next_point)

        if state == 'CornerReached':
            [v, omega] = basic_mov.rotate_to_target_point(polyline.get_next_point())

        if state == 'TargetReached':
            target_reached = True
            [v, omega] = [0, 0]

        myRobot.move([v, omega])


# close world by clicking
myWorld.close()
