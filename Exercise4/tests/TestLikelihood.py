# -*- coding: utf-8 -*-

__author__ = 'Ecki'


from datetime import datetime
from HTWG_Robot_Simulator_V1 import Robot as Robot, officeWorld as loadedWorld
from Exercise4.navigation import  Brushfire
from Exercise4.util import RobotLocation
from Exercise4.movements import CarrotDonkey
from Exercise4.util import Calculations as Calc


# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 5
set_robot_opt['y'] = 9
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)

# create instances
robot_loc = RobotLocation.RobotLocation(myRobot)
carrot_donkey = CarrotDonkey.CarrotDonkey(myRobot, myWorld)
occupancy_grid = myWorld.getOccupancyGrid(0.25)
brushfire = Brushfire.Brushfire(occupancy_grid, myRobot)
t_start = datetime.now()
brushfire.apply_brushfire(adjacency=8, safety_distance=0.25)
t_delta = (datetime.now() - t_start).total_seconds()
print 'Brushfire in: %0.5f seconds' % t_delta

myWorld.close()