import numpy as np

#trajectory from A to B
# call current coordinates in meters
xi = 1
yi = 1
zi = 1

# call desired coordinates in meters
xd = 2
yd = 2
zd = 2


current_pos = np.transpose(np.array([xi, yi, zi]))
desired_pos = np.transpose(np.array([xd, yd, zd]))
velocity = 0.1 #m/s


def next_pos(current_pos, desired_pos, velocity):
    vector = desired_pos - current_pos
    distance = np.linalg.norm(np.transpose(vector))
    unit_vector = vector/distance
    next_pos = current_pos + unit_vector*velocity
    return next_pos

