import numpy as np
import matplotlib.pyplot as plt
import math

X = [1,2,3,4,7]
Y = [2,7,6,1,10]
plt.scatter(X,Y)

def f(x):
    for i in range(len(X)-1):
        if x>=X[i] and x<X[i+1]:
            return Y[i] + (Y[i+1] - Y[i])*(x-X[i])/(X[i+1] - X[i])
    else:
        return Y[-1]


y = []
space = np.linspace(X[0],X[-1],100)
for x in space:
    y.append(f(x))
plt.plot(space, y, color="green")

y_np = np.interp(space, X, Y)
# plt.plot(space, y_np, color="red")
plt.show()
print(space)

