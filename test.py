import numpy as np
import matplotlib.pyplot as plt

x = np.matrix([0, 1, 2, 3])
y = np.matrix([-10,.2],[0.9],[2.1]])

print(x, y)

A = np.vstack([x, np.ones(len(x))]).T
print(A)

m, c = np.linalg.lstsq(A, y)[0]
print(m, c)