__author__ = 'Ecki'


from math import *
from HTWG_Robot_Simulator_V1 import emptyWorld
from HTWG_Robot_Simulator_V1 import Robot
from Exercise5.movements import CarrotDonkey as carrot_donkey
from Exercise5.tests import RobotLocationNoEst as RobotLocation

""" Init """

# Roboter in einer Welt positionieren:
world = emptyWorld.buildWorld()
robot = Robot.Robot()
world.setRobot(robot, 2, 8, 0)

world._sensorShow = False

# tell robo where he starts
[x0, y0, theta0] = robot.getTrueRobotPose()
robot.setOdoPose(x0, y0, theta0)

""" Main """


robot_loc = RobotLocation.RobotLocation(robot, world, None)
cd = carrot_donkey.CarrotDonkey(robot, world, robot_loc)

#define polyline
polyline = [[3, 8], [5, 8], [5, 6], [9, 6], [9, 8], [13, 5], [13, 8], [16, 8], [11, 8]]
world.drawPolyline(polyline)

cd.set_polyline(polyline, 0.5)

#while not robot_loc.robot_inside_point_tolerance(polyline[-1], 0.1):
while 1:

    #if cd.get_carrot_position()[0] > 12 and cd.carrot.p_end() != cd.carrot.p_last():
        move_commands = cd.next_movement_commands()
        robot.move(move_commands)


world.close()
