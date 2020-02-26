import numpy as np

x = 1
y = 1
z = 1
l = [[-1 / z, 0, x / z, x * y, -(1 + np.power(x, 2)), y],
            [0, -1 / z, y / z, 1 + np.power(y, 2), -x * y, -x]]
print(l)