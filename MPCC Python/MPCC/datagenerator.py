import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt

import parameters as param


# Author: Darina Abaffyová
# Created: 04/03/2020


def generate_track(num_steps):
    x_plus = np.array([7, -4.5, 11, 25.5, 16.5, 21.5])
    y_plus = np.array([10.5, -1.5, -1.5, 6.5, 9, 14.5])

    x_minus = np.array([7, -1.5, 11, 21, 13.5, 18.5])
    y_minus = np.array([7.5, 1.5, 1.5, 3.5, 6, 11.5])

    x = [0] * 6
    y = [0] * 6
    for i in range(6):
        x[i] = x_plus[i] - (x_plus[i] - x_minus[i]) / 2
        y[i] = y_plus[i] - (y_plus[i] - y_minus[i]) / 2

    # append the starting x,y coordinates
    x = np.r_[x, x[0]]
    y = np.r_[y, y[0]]

    x_plus = np.r_[x_plus, x_plus[0]]
    y_plus = np.r_[y_plus, y_plus[0]]

    x_minus = np.r_[x_minus, x_minus[0]]
    y_minus = np.r_[y_minus, y_minus[0]]

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x, y], s=0, per=True)
    tck_plus, u = interpolate.splprep([x_plus, y_plus], s=0, per=True)
    tck_minus, u = interpolate.splprep([x_minus, y_minus], s=0, per=True)

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)
    xi_plus, yi_plus = interpolate.splev(np.linspace(0, 1, num_steps), tck_plus)
    xi_minus, yi_minus = interpolate.splev(np.linspace(0, 1, num_steps), tck_minus)

    # plot the result
    fig, ax = plt.subplots(1, 1)
    ax.plot(xi, yi, 'or')
    ax.plot(xi_plus, yi_plus, '--b')
    ax.plot(xi_minus, yi_minus, '--g')
    plt.show()

    return [xi, yi]


def generate_racing_track(num_steps):
    # list = [0, 4, 8, 14, 19, 27, 30, 31, 37, 40]
    list = [0, 4, 8, 14, 19, 27, 30, 31, 37, 40, 43, 47, 51, 55, 59]
    # list.reverse()
    x = np.array(list)
    # y = np.array([0, 0, 0, 1, 5, 10, 5, 13, 5, 0, 0, 0, 0, 0, 0])
    # y = np.array([0, 0, 0, 0, 0, 10, 10, 10, 10, 10])
    # y = np.array([0, 1, 5, 10, 7, 5, 9, 11, 8, 4])
    y = np.array([0, 1, 5, 10, 7, 5, 9, 11, 8, 4, 0, -3, -9, -5, 0])

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x, y], s=0, per=False)

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)

    yu = []
    yl = []
    for i in range(len(xi)):
        yu.append(yi[i] + 3.5)  # param.track_width/2)
        yl.append(yi[i] - 3.5)  # param.track_width/2)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, 'or')
    # ax.plot(xi, yu, 'og')
    # ax.plot(xi, yl, 'og')
    # plt.show()

    return [xi, yi, yu, yl]


def offset(coordinates, distance):
    # coordinates = iter(coordinates)
    x1, y1 = coordinates[0]
    z = distance
    points = []

    for x2, y2 in coordinates:
        # tangential slope approximation
        try:
            slope = (y2 - y1) / (x2 - x1)
            # perpendicular slope
            pslope = - 1 / slope  # (might be 1/slope depending on direction of travel)
        except ZeroDivisionError:
            continue
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2

        sign = ((pslope > 0) == (x1 > x2)) * 2 - 1

        # if z is the distance to your parallel curve,
        # then your delta-x and delta-y calculations are:
        #   z**2 = x**2 + y**2
        #   y = pslope * x
        #   z**2 = x**2 + (pslope * x)**2
        #   z**2 = x**2 + pslope**2 * x**2
        #   z**2 = (1 + pslope**2) * x**2
        #   z**2 / (1 + pslope**2) = x**2
        #   z / (1 + pslope**2)**0.5 = x

        delta_x = sign * z / ((1 + pslope ** 2) ** 0.5)
        delta_y = pslope * delta_x

        points.append((mid_x + delta_x, mid_y + delta_y))
        x1, y1 = x2, y2

    return points


def generate_circular_track(num_steps):
    x = np.array([-7, 0, 7, 0])
    y = np.array([0, -7, 0, 7])
    upper_x = np.array([-5, 0, 5, 0])
    upper_y = np.array([0, -5, 0, 5])
    lower_x = np.array([-9, 0, 9, 0])
    lower_y = np.array([0, -9, 0, 9])

    # append the starting x,y coordinates
    x = np.r_[x, x[0]]
    y = np.r_[y, y[0]]
    upper_x = np.r_[upper_x, upper_x[0]]
    upper_y = np.r_[upper_y, upper_y[0]]
    lower_x = np.r_[lower_x, lower_x[0]]
    lower_y = np.r_[lower_y, lower_y[0]]

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x, y], s=0, per=True)
    tcku, u = interpolate.splprep([upper_x, upper_y], s=0, per=False)
    tckl, u = interpolate.splprep([lower_x, lower_y], s=0, per=False)

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)
    xu, yu = interpolate.splev(np.linspace(0, 1, num_steps), tcku)
    xl, yl = interpolate.splev(np.linspace(0, 1, num_steps), tckl)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, 'or')
    # ax.plot(xu, yu, '--g')
    # ax.plot(xl, yl, '--g')
    # plt.show()

    return [xi, yi, yu, yl]


def generate_linear_track(num_steps):
    x = np.arange(0, 50, 50 / num_steps)
    y = np.arange(0, 25, 25 / num_steps)

    upper = np.arange(2, 27, 25 / num_steps)
    lower = np.arange(-2, 23, 25 / num_steps)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(x, y, 'or')
    # ax.plot(x, upper, '--g')
    # ax.plot(x, lower, '--g')
    # plt.show()

    return [x, y, upper, lower]
