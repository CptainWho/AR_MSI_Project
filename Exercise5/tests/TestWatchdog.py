# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from HTWG_Robot_Simulator_V1 import Robot as Robot, officeWorld as loadedWorld
from Exercise5.util import RobotLocation
from Exercise5.util import Calculations as Calc
from Exercise5.obstacle_avoidance import Watchdog
from Exercise5.old_versions import CarrotDonkey_V1 as CarrotDonkey

# Create obstacleWorld and new Robot
myWorld = loadedWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 2
set_robot_opt['y'] = 7
set_robot_opt['theta'] = 0
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
polyline = [[3, 7], [6.5, 4.5], [9, 4.5], [10, 2], [12, 0], [17, 4]]

myWorld.drawPolyline(polyline)


# Follow polyline via carrot-donkey
carrot_donkey.setCarrotPosition(polyline[0])
for p_next in polyline:
    while carrot_donkey.carrot_pos != p_next:
        carrot_donkey.moveCarrotToPoint(p_next, 0.8)
        movement_commands = carrot_donkey.followCarrot()
        #movement_commands = w_dog.apply_watchdog(movement_commands, robot_loc.get_angle_from_robot_to_point(p_next))
        #myRobot.move(movement_commands)
        w_dog.move_robot(myRobot, movement_commands)

while not Calc.point_in_tol(robot_loc.get_robot_point(), polyline[-1], 0.2):
    movement_commands = carrot_donkey.followCarrot()
    w_dog.move_robot(myRobot, movement_commands)

myWorld.close()