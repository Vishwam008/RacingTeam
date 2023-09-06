
def swap(a, b):
    return tuple([b,a])
    print("Swap!")


def comp(a, b):
    c = 0
    while(c<len(a)-1 and a[c] == b[c]):
        c += 1
    if(a[c]<b[c]):
        return 1
    else:
        return 0


def sort(a, b):
    for i in range(len(a)-1):
        for j in range(i, len(a)):
            if comp(a[i], a[j]):
                a[i], a[j] = swap(a[i], a[j])
                b[i], b[j] = swap(b[i], b[j])


def upper_tri(a, b):
    l = len(a)
    for i in range(l-1):
        for j in range(1, l-i):
            r = a[j+i][i]/a[i][i]
            for k in range(l):
                a[j+i][k] = a[j+i][k] - a[i][k]*r
            b[j+i] = b[j+i] - b[i]*r


def gauss_elem(a, b):
    sort(a, b)
    upper_tri(a, b)
    n = len(a)
    x = list(0 for i in range(n))
    x[n-1] = b[n-1]/a[n-1][n-1]
    for i in range(n-2, -1, -1):
        s = 0
        for j in range(i+1, n):
            s += a[i][j]*x[j]
        x[i] = (b[i] - s)/a[i][i]
    return x


