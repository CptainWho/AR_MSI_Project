
__author__ = 'Ecki'

from math import *
import numpy as np
import PID as PID

class BasicMovement:
    def __init__(self, my_robot):
        self.robot = my_robot
        self.k_p = 0.6
        self.k_i = 0.001
        self.k_d = 0.8
        self.e_dist_old = 0
        self.int_e_dist_dt = 0
        self.dt = self.robot.getTimeStep()
        self.pid_fL = PID.PID(0.6, 0.001, 0.8, self.dt)
        self.pid_rotate = PID.PID(1.5, 0.01, 0.2, self.dt)


    def getRobotPos(self):
        return self.robot.getTrueRobotPose()

    def followLine(self, p1, p2, v):

        [x, y, theta] = self.getRobotPos()

        # Calculate distance e from point q to line g
        # line g = r1 + x * a
        # point q = [x, y]
        a = np.asarray(p2) - np.asarray(p1)
        r1 = np.asarray(p1)
        q = np.asarray([x, y])
        diff = q - r1
        e_dist = np.linalg.norm(np.cross(a, diff)) / np.linalg.norm(a)

        # Check if robot is on the left or on the right side of the line
        # Cross-product of the direction vectors of line g: (p2 - p1) and line between p1 and point q: (q - p1)
        # Right-hand rule determines the orientation of the resulting imaginary z-axis:
        # Positive: robot is on the right side of the line
        # Negative: robot is on the left side of the line
        z = a[1] * (x - p1[0]) - a[0] * (y - p1[1])

        e_dist *= -np.sign(z)

        omega = self.pid_fL.control(e_dist)

        return [v, omega]


    # rotates robot to the specified point with an angle tolerance
    def rotateToTargetPoint(self, point, angle_tol):

        # target direction
        [x, y, theta] = self.getRobotPos()
        theta_target = atan2(point[1] - y, point[0] - x)

        omega = -self.pid_rotate.control(self.diff(theta, theta_target))
        v = 0
        return [v, omega]

    # calculate the angle difference from theta to theta_target
    # positive is defined as counterclockwise
    def diff(self, theta, theta_target):
        return (theta_target - theta + pi) % (2 * pi) - pi