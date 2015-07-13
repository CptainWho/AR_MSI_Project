# -*- coding: utf-8 -*-
__author__ = 'Ecki'

# Standard library imports
from math import *
# Local imports
from HTWG_Robot_Simulator_V1 import Robot as Robot, officeWorldWithDynObstacles_V1_3 as loadedWorld
from Exercise5.state_machine import StateMachine, TransitionsBoxLoc as Transitions
from Exercise5.obstacle_avoidance import HistogramGrid
from Exercise5.obstacle_avoidance import PolarHistogram
from Exercise5.util import RobotLocation
from Exercise5.movements import CarrotDonkey as CarrotDonkey
from Exercise5.obstacle_avoidance import Watchdog
from Exercise5.navigation import PathScheduler
from Exercise5.util import Calculations as Calc
from Exercise5.localization import BoxLocator

# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
rooms = myWorld.getRooms()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
#set_robot_opt['x'] = 2
#set_robot_opt['y'] = 6
set_robot_opt['x'] = 12
set_robot_opt['y'] = 10
set_robot_opt['theta'] = pi/2
myWorld.setRobot(**set_robot_opt)

# define polyline
polyline0 = [[12.0, 10.0], [12.25, 10.25], [12.25, 10.5], [12.25, 10.75], [12.25, 11.0], [12.5, 11.25], [12.75, 11.5], [13.0, 11.5], [13.25, 11.5], [13.5, 11.5], [13.75, 11.5], [14.0, 11.5], [14.25, 11.5], [14.5, 11.25], [14.5, 11.0], [14.5, 10.75], [14.5, 10.5], [14.5, 10.25], [14.5, 10.0], [14.5, 9.75], [14.5, 9.5], [14.5, 9.25], [14.5, 9.0]]
polyline1 = [[14.495627687467874, 9.0773982406801164], [14.5, 4.5]]
polyline2 = [[14.504105999744633, 4.5932562353406832], [14.254105999744633, 7.5932562353406832], [14.254105999744633, 10.593256235340682], [14.504105999744633, 11.343256235340682], [14.004105999744633, 11.843256235340682]]
line_no = 1


# Set up StateMachine
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld)

robot_loc = RobotLocation.RobotLocation(myRobot)
#histogram = HistogramGrid.HistogramGrid(5, 5, cell_size=0.1, hist_threshold=5.0)
polar_hist = PolarHistogram.PolarHistogram(myRobot, robot_loc)
w_dog = Watchdog.Watchdog(robot_loc)
path_sched = PathScheduler.PathScheduler(myWorld)
transitions = Transitions.Transitions(myRobot, carrot_donkey, path_sched)
state_machine = StateMachine.StateMachine(transitions)
box_loc = BoxLocator.BoxLocator(robot_loc)

A = [0, 0]
B = [4, 0]
C = [0, 2]
D = [4, 2]

box = BoxLocator.Box(A)
box.add_point(B)
box.add_point(C)
box.add_point(D)

print box.pos

# close world by clicking
myWorld.close()
