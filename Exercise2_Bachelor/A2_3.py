# -*- coding: utf-8 -*-
__author__ = 'Ecki'


import MatrixTansformation
from numpy import *

mt = MatrixTansformation

""" Teilaufgabe a) """
print "a)"

t_OA = array([[1], [1], [0]])
theta_OA = 0
t_OB = array([[3], [2], [0]])
theta_OB = 30*pi/180

T_OA = mt.transform(t_OA, mt.rotz(theta_OA))
T_OB = mt.transform(t_OB, mt.rotz(theta_OB))

print "T_OA ="
print T_OA
print "T_OB ="
print T_OB

""" Teilaufgabe b) """
print "b)"
P_B = array([[1], [1], [0], [1]])
P_O = dot(T_OB, P_B)

print "P_O ="
print P_O

""" Teilaufgabe c) """
print "c)"

T_AB = dot(linalg.inv(T_OA), T_OB)

print "T_AB ="
print T_AB

""" Teilaufgabe d) """
print "d)"

P_A = dot(T_AB, P_B)

print "P_A ="
print P_A

""" Teilaufgabe e) """
print "e)"

print dot(T_AB, P_A)
