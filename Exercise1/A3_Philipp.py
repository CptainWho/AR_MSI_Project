""" Module
Module Description:
"""

__project__ = 'Aufgabenblatt 1'
__module__  = 'A3'
__author__  = 'Philipp Lohrer'
__email__   = 'plohrer@htwg-konstanz.de'
__date__    = '23.04.2015'

__version__ = '1.0'

# Imports
#################################################################
# Standard library imports
import numpy as np
from math import *
from numbers import Number
from datetime import datetime
# Local imports
from HTWG_Robot_Simulator_V1 import (emptyWorld, Robot)
#################################################################

def followLine(p1, p2, **kwargs):
    """
    :param p1: tuple(x1,y1)
    :param p2: tuple(x2,y2)
    :param kwargs: speed, controller
    :return: -
    """

    # Controller parameters
    k_p = 0.2
    k_d = 0.5
    k_i = 0.1
    e_old = 0
    e_sum = 0
    t_old = datetime.now()

    # Set tolerance for reaching end of line (p2)
    tol = 1
    if 'tolerance' in kwargs:
        if isinstance(kwargs['tolerance'], Number):
            tol = kwargs['tolerance']
    print('tol = %0.2f' % tol)

    # Set robot speed to default value or check if speed is passed
    v = 0.5
    if 'speed' in kwargs:
        if isinstance(kwargs['speed'], Number):
            v = kwargs['speed']
    print('v = %0.2f' % v)

    # Set controller to default or check if another controller is passed
    c = 'P'
    controllers = ['P', 'PD', 'PI', 'PID']
    if 'controller' in kwargs:
        if kwargs['controller'] in controllers:
            c = kwargs['controller']
    print('Controller = %s' % c)

    # Draw the line and display it
    myWorld.drawPolyline((p1, p2))

    # Get estimated position and orientation of the robot
    [x, y, theta] = myRobot.getOdoPose()

    # Follow the line until the robot reaches p2 with tolerance tol
    while outOfTol((x, y), p2, tol):
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
        print('e: %0.5f' % e)


        # Check if robot is on the left or on the right side of the line
        # Cross-product of the direction vectors of line g: (p2 - p1) and line between p1 and point q: (q - p1)
        # Right-hand rule determines the orientation of the resulting imaginary z-axis:
        # Positive: robot is on the right sid eof the line
        # Negative: robot is on the left side of the line
        z = a[1] * (x - p1[0]) - a[0] * (y - p1[1])

        # Run control mode according to selected controller
        if c == 'P':
            # P control
            omega = k_p * e * np.sign(z)
            myRobot.move((v, omega))
        elif c == 'PD':
            # PD control
            t = datetime.now()
            dt = (t - t_old).total_seconds()
            e_dt = (e - e_old) / float(dt)
            omega = (k_p * e + k_d * e_dt) * np.sign(z)
            myRobot.move((v, omega))
            e_old = e
            t_old = t
        elif c == 'PI':
            # PI control
            t = datetime.now()
            dt = (t - t_old).total_seconds()
            e_sum += e
            omega = (k_p * e + k_i * e_sum * dt) * np.sign(z)
            myRobot.move((v, omega))
            t_old = t
        elif c == 'PID':
            # PID control
            t = datetime.now()
            dt = (t - t_old).total_seconds()
            e_sum += e
            e_dt = (e - e_old) / float(dt)
            omega = (k_p * e + k_i * e_sum * dt + k_d * e_dt) * np.sign(z)
            myRobot.move((v, omega))
            e_old = e
            t_old = t


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
# Tell robot where he starts
myRobot.setOdoPose(x0, y0, theta0)

# Set noise to zero
myRobot._k_d = 0
myRobot._k_drift = 0
myRobot._k_theta = 0

# Define line and let robot follow it
p1 = [x0, y0+2]
p2 = [x0+15, y0-2]
followLine(p1, p2, speed=0.2, controller='PID', tolerance=0.2)

# close world by clicking
myWorld.close()
