#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Ecki'


import numpy as np

""" Liefert eine 2D-Rotationsmatrix mit Drehwinkel theta zurueck. """
def rot(theta):
    rot2d = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
    return rot2d

""" Liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drechachse x zurueck. """
def rotx(theta):
    rot3d = np.array([[1, 0, 0],
                      [0, np.cos(theta), -np.sin(theta)],
                      [0, np.sin(theta), np.cos(theta)]])
    return rot3d

""" Liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drechachse y zurueck. """
def roty(theta):
    rot3d = np.array([[np.cos(theta), 0, np.sin(theta)],
                      [0, 1, 0],
                      [-np.sin(theta), 0, np.cos(theta)]])
    return rot3d

""" Liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drechachse z zurueck. """
def rotz(theta):
    rot3d = np.array([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta), np.cos(theta), 0],
                      [0, 0, 1]])
    return rot3d

""" Wandelt die Rotationsmatrix r in eine homogene Transfromationsmatrix um und liefert diese zurueck. """
def rot2trans(r):
    # r und 0-Matrix nebeneinander anordnen
    row1 = np.hstack((r, np.zeros((np.shape(r)[0], 1))))
    # 0-Matrix und 1 nebeneinander anordnen
    row2 = np.hstack((np.zeros((1, np.shape(r)[1])), np.array([[1]])))
    # Beide Reihen untereinander zurueckgeben
    return np.vstack((row1, row2))

""" Liefert eine homogene Translationsmatrix mit Translation t zurueck. t ist ein Tupel der Groesse 2 bzw. 3 für den 2D- bzw. 3D- Fall. """
def trans(t):
    # Einheitsmatrix und t-Vektor nebeneinander anordnen
    row1 = np.hstack((np.eye(np.shape(t)[0]), t))
    # 0-Matrix und 1 nebeneinander anordnen
    row2 = np.hstack((np.zeros((1, np.shape(t)[0])), np.array([[1]])))
    # Beide Reihen untereinander zurueckgeben
    return np.vstack((row1, row2))

""" Liefert eine homogene Transformationsmatrix mit Translation t und der Rotation aus der Rotationsmatrix rotMat zurueck. """
def transform(t, rotMat):
    t_c_b = rot2trans(rotMat)
    t_a_c = trans(t)
    t_a_b = np.dot(t_a_c, t_c_b)
    return t_a_b