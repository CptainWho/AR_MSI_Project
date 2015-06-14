# -*- coding: utf-8 -*-
__author__ = 'Ecki'

from Exercise3.robot_navigation import AStarAlgo

list = AStarAlgo.OpenList()

print list.not_empty()

for i in range(10):
    list.push([i, 10-i], 1-i/10.0, 2)

print list.not_empty()

print list.list
found = list.find([5,5])
index = list.get_index([5,5])
print list.pop()
print found
print index




