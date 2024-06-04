import numpy as np

a = np.empty([1], dtype=int)
print(a)
a = np.append(a, [3])
b = np.min(a)
print(a)
print(b)