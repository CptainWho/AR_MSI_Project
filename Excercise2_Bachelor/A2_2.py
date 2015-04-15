__author__ = 'Ecki'
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import MatrixTansformation
from numpy import *

mt = MatrixTansformation

t_AB = array([[-2], [0], [0]])
theta_AB = pi
t_BC = array([[-4], [-1], [0]])
theta_BC = (pi/2)*3
t_AC = array([[2], [1], [0]])
theta_AC = pi/2

""" Teilaufgabe a) """
print "a)"

T_AB = mt.transform(t_AB, mt.rotz(theta_AB))
T_BC = mt.transform(t_BC, mt.rotz(theta_BC))
T_AC = mt.transform(t_AC, mt.rotz(theta_AC))

print "T_AB ="
print T_AB
print "T_BC ="
print T_BC
print "T_AC ="
print T_AC

""" Teilaufgabe b) """
print "b)"

print "T_AB * T_BC ="
print dot(T_AB, T_BC)
print "T_AC ="
print T_AC

""" Teilaufgabe c) """
print "c)"
t_CA = array([[-1], [2], [0]])
theta_CA = (pi/2)*3

T_CA = mt.transform(t_CA, mt.rotz(theta_CA))

print "T_CA ="
print T_CA
print "inv(T_AC) ="
print linalg.inv(T_AC)

""" Teilaufgabe d) """
print "d)"
P_B = array([[-3], [1], [0], [1]])
P_A = array([[1], [-1], [0], [1]])

print "P_B ="
print P_B
print "P_A ="
print P_A
print "T_AB * P_B ="
print dot(T_AB, P_B)

""" Teilaufgabe d) """
print "d)"
P_C = array([[-2], [1], [0], [1]])

print "P_A ="
print P_A
print "P_C ="
print P_C
print "T_AB * P_B ="
print dot(T_CA, P_A)