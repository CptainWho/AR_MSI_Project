__author__ = 'Ecki'
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import MatrixTansformation
from numpy import *

mt = MatrixTansformation

xR = 2
yR = 1
r = 0.1
h = 0.2
d = 0.3
l1 = 0.5
l2 = l1
theta = 30*pi/180
alpha = 10*pi/180
beta1 = 20*pi/180
beta2 = -10*pi/180

t0 = array([[0], [0], [0]])


t_OR = array([[xR], [yR], [r]])
theta_OR_z = theta

t_RA1 = array([[d], [0], [h]])
theta_RA1_z = alpha
theta_RA1_y = -beta1

t_A1A2 = array([[l1], [0], [0]])
theta_A1A2_y = -beta2

P_A2 = array([[l2], [0], [0], [1]])

T_OR = mt.transform(t_OR, mt.rotz(theta_OR_z))
T_RA1 = dot(mt.transform(t_RA1, mt.rotz(theta_RA1_z)), mt.transform(t0, mt.roty(theta_RA1_y)))
T_A1A2 = mt.transform(t_A1A2, mt.roty(theta_A1A2_y))

T_OA2 = dot(dot(T_OR, T_RA1), T_A1A2)

P_O = dot(T_OA2, P_A2)

print "T_OA2 ="
print T_OA2
print "P_O ="
print P_O

print "Probe:"
x = xR + d*cos(theta) + l1*cos(beta1)*cos(theta+alpha) + l2*cos(beta1+beta2)*cos(theta+alpha)
y = yR + d*sin(theta) + l1*cos(beta1)*sin(theta+alpha) + l2*cos(beta1+beta2)*sin(theta+alpha)
z = r + h + l1*sin(beta1) + l2*sin(beta1+beta2)

print "x =", x
print "y =", y
print "z =", z

