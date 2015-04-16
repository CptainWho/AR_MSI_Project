__author__ = 'Ecki'

import numpy as np

def vectorAngle(vector_a, vector_b):
    vector_a = vector_a.transpose()
    dot = np.dot(vector_a, vector_b)
    product = np.linalg.norm(vector_a) * np.linalg.norm(vector_b)
    div = dot / product
    print div
    return np.arccos(div)

a = np.array([[1], [0], [0]])
b = np.array([[-1], [1], [0]])

print vectorAngle(a,b) * 180 / np.pi