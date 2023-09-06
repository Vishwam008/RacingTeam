import matplotlib.pyplot as plt
import Gaussian
import math
import numpy as np

def partition(coeff):
    B=[]
    C=[]
    D=[]
    print(len(coeff))
    for i in range(0, len(coeff), 3):
        B.append(coeff[i])
        C.append(coeff[i+1])
        D.append(coeff[i+2])
    return tuple([B, C, D])

p = [[1, 10], [2, 7], [3, 5], [4, 1], [6,2], [9,10], [10,3]]
n=len(p)-1

plt.scatter(list(a[0] for a in p), list(a[1] for a in p))

l_a=list(0 for el in range(n*3))
m=[]
l_b=list(0 for el in range(n*3))
count = 0

l_a[1] = 1
l_b[count] = 0
count+=1
m.append(l_a)

for i in range(n-1): 
    l_a=list(0 for el in range(n*3))
    c=i*3
    l_a[c] = (p[i+1][0]-p[i][0])
    l_a[c+1] = (p[i+1][0]-p[i][0])**2
    l_a[c+2] = (p[i+1][0]-p[i][0])**3
    l_b[count] = (p[i+1][1]-p[i][1])
    count += 1
    m.append(l_a)

    l_a=list(0 for el in range(n*3))
    l_a[c+1] = 1
    l_a[c+2] = 3*(p[i+1][0] - p[i][0])
    l_a[c+4] = -1
    l_b[count] = 0
    count += 1
    m.append(l_a)

    l_a=list(0 for el in range(n*3))
    l_a[c] = 1
    l_a[c+1] = 2*(p[i+1][0] - p[i][0])
    l_a[c+2] = 3*(p[i+1][0] - p[i][0])**2
    l_a[c+3] = -1
    l_b[count] = 0
    count += 1
    m.append(l_a)
    
l_a=list(0 for el in range(n*3))
l_a[3*n-3] = (p[n][0]-p[n-1][0])
l_a[3*n-2] = (p[n][0]-p[n-1][0])**2
l_a[3*n-1] = (p[n][0]-p[n-1][0])**3
l_b[count] = (p[n][1]-p[n-1][1])
count += 1
m.append(l_a)

l_a=list(0 for el in range(n*3))
l_a[-2] = 2
l_a[-1] = (p[n][0]-p[n-1][0])
l_b[count] = 0
count+=1
m.append(l_a)

for item in m:
    print(item)
print(l_b)

A=[]
for i in range(len(p)):
    A.append(p[i][1])

coeff = Gaussian.gauss_elem(m, l_b)
print(coeff)
B, C, D = partition(coeff)

def f(x):
    for i in range(len(p)-1):
        if x>=p[i][0] and x<p[i+1][0]:
            return A[i] + B[i] * (x - p[i][0]) + C[i] * (x - p[i][0])**2 + D[i] * (x - p[i][0])**3
    else:
        return p[-1][1]

space = np.linspace(p[0][0], p[-1][0], 100)
y=[]
for i in space:
    y.append(f(i))
plt.plot(space, y, color="red")
plt.show()