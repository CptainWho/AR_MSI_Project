__author__ = 'Ecki'
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import MatrixTansformation
from numpy import *

mt = MatrixTansformation

P_B = array([[2], [1], [1]])
t = array([[2], [2]])
theta = pi/2

# T_C_B = mt.rot2trans(mt.rot(theta))
# T_A_C = mt.trans(t)
# T_A_B = dot(T_A_C, T_C_B)
# oder:

T_A_B = mt.transform(t,mt.rot(theta))

P_A = dot(T_A_B, P_B)

print P_B
print P_A
