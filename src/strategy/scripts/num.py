import numpy as np

l = np.zeros(5)

l = np.delete(l, 1)
a = 5 
l = np.append(l, a)


l = np.delete(l, 1)
a = 5 
l = np.append(l, a)

add = sum(l) 
print(add)