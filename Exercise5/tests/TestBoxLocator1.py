# -*- coding: utf-8 -*-
__author__ = 'Ecki'

# Standard library imports
from math import *
# Local imports
from HTWG_Robot_Simulator_V1 import Robot as Robot, emptyWorld as loadedWorld
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
set_robot_opt['y'] = 9
set_robot_opt['theta'] = pi/7.0*5
myWorld.setRobot(**set_robot_opt)


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

v = 1
omega = 0
for i in xrange(10):
    w_dog.move_robot(myRobot, [v, omega])
    print box_loc.sense_box_points()


# close world by clicking
myWorld.close()
