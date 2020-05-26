import numpy as np
from scipy import interpolate
import parameters as param
import matplotlib.pyplot as plt


# Author: Darina Abaffyová
# Created: 04/03/2020
# Last updated: 23/03/2020


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
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, 'or')
    # ax.plot(xi_plus, yi_plus, '--b')
    # ax.plot(xi_minus, yi_minus, '--g')
    # plt.show()

    return [xi, yi]


def generate_racing_track(num_steps):
    x = np.array([0, 2, 4, 8, 14, 19, 25])
    y = np.array([0, 1, 5, 10, 2, 13, 5])
    # upper = np.array([1.5, 2.5, 6.5, 11.5, 3.5, 14.5, 6.5])
    # lower = np.array([-1.5, -0.5, 3.5, 8.5, 0.5, 11.5, 3.5])

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x, y], s=0, per=False)
    # tcku, u = interpolate.splprep([x, upper], s=0, per=False)
    # tckl, u = interpolate.splprep([x, lower], s=0, per=False)

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)
    # xu, yu = interpolate.splev(np.linspace(0, 1, num_steps), tcku)
    # xl, yl = interpolate.splev(np.linspace(0, 1, num_steps), tckl)

    yu = []
    yl = []
    for i in range(len(xi)):
        yu.append(yi[i] + 2)
        yl.append(yi[i] - 2)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, 'or')
    # ax.plot(xi, yu, '--g')
    # ax.plot(xi, yl, '--g')
    # plt.show()

    return [xi, yi, yu, yl]


def generate_circular_track(num_steps):
    x = np.array([-3, 0, 3, 0])
    y = np.array([0, -3, 0, 3])

    # append the starting x,y coordinates
    x = np.r_[x, x[0]]
    y = np.r_[y, y[0]]

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x, y], s=0, per=True)

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, 'or')
    # plt.show()

    return [xi, yi]


def generate_linear_track(num_steps):
    y = np.arange(0, 25, 25 / num_steps)
    x = np.arange(0, 20, 20 / num_steps)

    upper = np.arange(1.5, 26.5, 25 / num_steps)
    lower = np.arange(-1.5, 23.5, 25 / num_steps)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(x, y, 'or')
    # ax.plot(x, upper, '--g')
    # ax.plot(x, lower, '--g')
    # plt.show()

    return [x, y, upper, lower]
