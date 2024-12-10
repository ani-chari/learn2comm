import numpy as np
from scipy.interpolate import CubicSpline
import random
import math

def make_path(start, goal):
    m = (goal[1] - start[1]) / (goal[0] - start[0])
    straight = np.array([1, m]) / np.linalg.norm(np.array([1, m])) * np.sign(goal - start)
    perp = np.array([1, -1/m]) / np.linalg.norm(np.array([1, -1/m]))
    points = []
    points.append(start)
    p = start.copy()
    n = random.randint(3, 7) # TODO: tune
    delta = np.linalg.norm(goal - start) / n
    for i in range(n - 1): # TODO: tune
        p += straight * delta
        eps = random.uniform(-3, 3) # TODO: tune
        pt = p + perp * eps
        points.append(pt)
    points.append(goal)
    points = np.array(sorted(sorted(points,key=lambda e:e[1]),key=lambda e:e[0]))
    if points[0][0] > points[1][0]:
        points.reverse()
    spline = CubicSpline([x[0] for x in points], [x[1] for x in points])

    # parameterize spline
    path = {0: start}
    dx = 0.1
    t = 0
    dt = 0.1
    x = min(start[0], goal[0])
    arc_len = 0
    len_max = 1 # TODO: tune; length traveled in dt seconds
    while x < max(goal[0], start[0]):
        x += dx
        arc_len += math.sqrt(1 + spline.derivative()(x)**2) * dx
        if arc_len >= len_max:
            arc_len = 0
            t += dt
            path[round(t, 1)] = np.array([x, spline(x)])
    path[round(t + dt, 1)] = goal

    return path