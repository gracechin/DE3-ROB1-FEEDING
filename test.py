import numpy as np
import matplotlib.pyplot as plt

# Random data
N = [[255.0, 243.0, 0.0], [255.0, 248.0, 0.0]]
M = [[0.188, -0.308, 0.488], [0.188, -0.308, 0.488]]
#input = np.random.random((N,M))
# print input 

# Setup matrices
m = np.shape(N)[0]
X = np.matrix(np.hstack((np.ones((m,1)), N))).T
Y = np.matrix(M).T
print("X: ", X, "Y: ", Y)

# Solve for projection matrix
print X.T.dot(X)
# print dot(X.T).dot(Y)
# p_mat = np.linalg.inv( X.T.dot(X) ).dot(X.T).dot(Y)
p_mat = np.array(np.linalg.inv( X.T.dot(X) ))

print "pmat: ", p_mat
print("M1", p_mat[0][0]+p_mat[0][1]*N[0][0])

# Find regression line
# xx = np.linspace(0, 1, 2)
# yy = np.array(p_mat[0] + p_mat[1] * xx)

# # Plot data, regression line
# plt.figure(1)
# plt.plot(xx, yy.T, color='b')
# plt.scatter(input[:,0], input[:,1], color='r')
# plt.show()