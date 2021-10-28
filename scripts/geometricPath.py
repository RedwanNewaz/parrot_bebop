import numpy as np
import math
# Online Python compiler (interpreter) to run Python online.
# Write Python 3 code in this online editor and run it.
def circle():
    theta = np.arange(0, 2*math.pi, math.pi/16)
    r = 2
    x0, y0 = 0, 0
    X, Y = [], []
    for q in theta:
        x = x0 + r * math.cos(q)
        y = y0 + r * math.sin(q)
        X.append(x)
        Y.append(y)


    X.append(X[0])
    Y.append(Y[0])
    return X, Y

def sweepPath(origin, width, height, delta):
    A = np.array(origin)
    B = np.array([origin[0]+ width, origin[1]])
    C = np.array([origin[0], origin[1] + height])
    D = np.array([origin[0] + width, origin[1] + height])

    theta = np.arange(0, 1, delta)

    X, Y = [], []

    for i, q in enumerate(theta):
        lowerPoint = A * (1 - q) + B * ( q)
        upperPoint = C * (1 - q) + D * ( q)

        if i %2 == 0:
            X.append(lowerPoint[0])
            X.append(upperPoint[0])
            Y.append(lowerPoint[1])
            Y.append(upperPoint[1])
        else:
            X.append(upperPoint[0])
            X.append(lowerPoint[0])
            Y.append(upperPoint[1])
            Y.append(lowerPoint[1])
    return X, Y

def interpolate_path(path, segments = 200):
    '''
    :param path: sweep path
    :return: intermediate goal location for the receding horizon controller
    '''
    dr = (np.diff(path[:, 0]) ** 2 + np.diff(path[:, 1]) ** 2) ** .5  # segment lengths
    r = np.zeros_like(path[:, 0])
    r[1:] = np.cumsum(dr)  # integrate path

    r_int = np.linspace(0, r.max(), segments)  # regular spaced path
    x_int = np.interp(r_int, r, path[:, 0])  # interpolate
    y_int = np.interp(r_int, r, path[:, 1])
    return np.array([x_int, y_int]).T

def interpolate(X, Y, segments):
    path = np.array(list(zip(X, Y)))
    path = interpolate_path(path, segments = segments)
    return path[:, 0].tolist(), path[:, 1].tolist()


def printArray(arr):
    strArr = map(lambda x: "{:.3f}".format(x), arr)
    arr = map(float, strArr)
    print(list(arr))

if __name__ == '__main__':
    # X, Y = circle()

    X, Y = sweepPath(origin = (0, 0), width = 4, height = 4, delta = 0.1)
    X, Y = interpolate(X, Y, segments = 200)

    print("wp_x", end=": ")
    printArray(X)
    print("wp_y", end=": ")
    printArray(Y)