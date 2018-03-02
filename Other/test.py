import numpy as np

# Random data
A = np.array([[255.0, 243.0, 0.0], [255.0, 248.0, 0.0]])
B = np.array([[0.188, -0.308, 0.488], [0.188, -0.408, 0.488]])

print("A", A)
print("B", B)

m_list = [] # m for x, y, z
c_list = [] # c for x, y, z

for dimension in range(3):
	a = []
	b = []
	for point_index in range(len(A)):
		a.append(A[point_index][dimension])
		b.append(B[point_index][dimension])
	np.array(a)
	np.array(b)
	a = np.vstack([a, np.ones(len(a))]).T
	m, c = np.linalg.lstsq(a, b)[0]
	m_list.append(m)
	c_list.append(c)
scale=[m_list, c_list]
print(scale)

input_pt = [255.0, 246.0, 0.0]
output_pt = []
for di in range(3):
	output_pt.append(input_pt[di]*m_list[di]+c_list[di])

print(output_pt)