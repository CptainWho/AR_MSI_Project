# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from HTWG_Robot_Simulator_V1 import Robot as Robot, emptyWorld as loadedWorld
from Exercise5.util import RobotLocation
from Exercise5.util import Calculations as Calc
from Exercise5.obstacle_avoidance import Watchdog
from Exercise5.movements import CarrotDonkey as CarrotDonkey
from math import *

# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 8
set_robot_opt['y'] = 1
# set_robot_opt['x'] = 2
#set_robot_opt['y'] = 3
set_robot_opt['theta'] = 0+pi/2.0
myWorld.setRobot(**set_robot_opt)

# create instances
robot_loc = RobotLocation.RobotLocation(myRobot)
w_dog = Watchdog.Watchdog(robot_loc)
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld)

# define points
start_point = robot_loc.get_robot_point()
end_point = [10, 4]
myWorld.addBox(end_point[0], end_point[1])

# Find shortest path from start_point to end_point
polyline = [[6, 2], [5, 3], [5, 15], [10, 16], [13, 6], [9, 13], [9, 14], [9, 8], [7, 14], [2, 15], [6, 2], [5, 3], [5, 15], [10, 16], [13, 6], [9, 13], [9, 14], [9, 8], [7, 14], [2, 15]]
#polyline = [[4, 2], [12, 5], [9, 14], [2, 15]]

myWorld.drawPolyline(polyline)

carrot_donkey.set_polyline(polyline, 0.5)

while 1:
    movement_commands = carrot_donkey.next_movement_commands()
    w_dog.move_robot(myRobot, movement_commands)

myWorld.close()