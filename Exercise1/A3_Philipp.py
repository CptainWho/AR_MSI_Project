""" Module
Module Description:
"""

__project__ = 'Aufgabenblatt 1'
__module__  = 'A3'
__author__  = 'Philipp Lohrer'
__email__   = 'plohrer@htwg-konstanz.de'
__date__    = '21.04.2015'

__version__ = '0.1'

# Imports
#################################################################
# Standard library imports
import numpy as np
from math import *
from numbers import Number
# Local imports
from HTWG_Robot_Simulator_V1 import (emptyWorld, Robot)
#################################################################

def followLine(p1, p2, **kwargs):
    """
    :param p1: tuple(x1,y1)
    :param p2: tuple(x2,y2)
    :return:
    """

    # Set robot speed to default value or check if speed is passed
    v = 0.5
    if 'speed' in kwargs:
        if isinstance(kwargs['speed'], Number):
            v = kwargs['speed']
    print('v = %0.2f' % v)

    # Set controller to default or check if another controller is passed
    c = 'P'
    controllers = ['P', 'PD', 'PID']
    if 'controller' in kwargs:
        if kwargs['controller'] in controllers:
            c = kwargs['controller']
    print('Controller = %s' % c)

    # Draw the line and display it
    myWorld.drawPolyline((p1, p2))

    # Get estimated position and orientation of the robot
    [x, y, theta] = myRobot.getOdoPose()

    # Follow the line until the robot reaches p2 with tolerance tol
    while outOfTol((x, y), p2, 1):
        # Get estimated position and orientation of the robot
        [x, y, theta] = myRobot.getOdoPose()

        # Calculate distance e from point q to line g
        # line g = r1 + x * a
        # point q = [x, y]
        a = np.asarray(p2) - np.asarray(p1)
        r1 = np.asarray(p1)
        q = np.asarray([x, y])
        diff = q - r1
        e = np.linalg.norm(np.cross(a, diff)) / np.linalg.norm(a)

        # Check if robot is on the left or on the right side of the line
        # Cross-product of the direction vectors of line g: (p2 - p1) and line between p1 and point q: (q - p1)
        # Right-hand rule determines the orientation of the resulting imaginary z-axis:
        # Positive: robot is on the right sid eof the line
        # Negative: robot is on the left side of the line
        z = a[1] * (x - p1[0]) - a[0] * (y - p1[1])

        if c == 'P':
            # P control
            k_p = 0.1
            omega = k_p * e * np.sign(z)
            myRobot.move((v, omega))
        elif c == 'PD':
            # PD control
            break

# check whether point in within tolerance of target point
def outOfTol(p, p_target, tol):
    sqrDist = (p[0] - p_target[0])**2 + (p[1] - p_target[1])**2
    sqrTol = tol**2
    if sqrDist > sqrTol:
        return True
    else:
        return False



# Create empty World and new Robot
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
# Place Robot in World
set_robot_opt = {}
set_robot_opt['robot'] = myRobot
set_robot_opt['x'] = 1
set_robot_opt['y'] = 10
set_robot_opt['theta'] = 0
myWorld.setRobot(**set_robot_opt)



[x0, y0, theta0] = myRobot.getTrueRobotPose()
# tell robo where he starts
myRobot.setOdoPose(x0, y0, theta0)

p1 = [x0, y0-2]
p2 = [x0+10, y0-2]
followLine(p1, p2, speed=0.2, controller='P')

print myRobot.getOdoPose()

# close world by clicking
myWorld.close()
