from scipy import optimize


def radius(tup):
    c, d =tup
    B = [-11+29*c+13*d, 7-13*c-6*d, -5-c-d]
    C = [30-45*c-20*d, -12+3*c+d, 1]
    D = [-14+16*c+7*d, 4+2*c+2*d, 1]
    s = 0
    
    for i in range(3):
        y1 = B[i]
        y2 = 2*C[i]
        r = ((1+y1**2)**1.5)/abs(y2)
        s += r
    return s

x0 = [0, 2]
res = optimize.minimize(radius, x0)
print(res.x)