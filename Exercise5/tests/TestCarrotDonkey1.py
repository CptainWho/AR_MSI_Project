__author__ = 'Ecki'


from math import *
from HTWG_Robot_Simulator_V1 import emptyWorld
from HTWG_Robot_Simulator_V1 import Robot
from Exercise5.movements import CarrotDonkey as carrot_donkey
from Exercise5.util import RobotLocation

""" Init """

# Roboter in einer Welt positionieren:
world = emptyWorld.buildWorld()
robot = Robot.Robot()
world.setRobot(robot, 5, 5, -pi/2)

# tell robo where he starts
[x0, y0, theta0] = robot.getTrueRobotPose()
robot.setOdoPose(x0, y0, theta0)

""" Main """

cd = carrot_donkey.CarrotDonkey(robot, world)
robot_loc = RobotLocation.RobotLocation(robot)

#define polyline
polyline = [[1, 6], [2, 8], [10, 8], [10, 6], [18, 6]]
world.drawPolyline(polyline)

cd.set_polyline(polyline, 2.5)

#while not robot_loc.robot_inside_point_tolerance(polyline[-1], 0.1):
while 1:
    move_commands = cd.next_movement_commands()
    robot.move(move_commands)


world.close()
