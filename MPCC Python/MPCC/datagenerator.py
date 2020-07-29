import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate

import parameters as param


# Author: Darina Abaffyová
# Created: 04/03/2020

def generate_linear_track(num_steps):
    x = np.arange(0, 50, 50 / num_steps)
    y = np.arange(0, 50, 50 / num_steps)

    left = np.arange(param.track_width, param.track_width + 50, 50 / num_steps)
    right = np.arange(-param.track_width, -param.track_width + 50, 50 / num_steps)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(x, y, 'or')
    # ax.plot(x, left, '--g')
    # ax.plot(x, right, '--g')
    # plt.show()

    return [x, y, x, left, x, right]


def generate_circular_track(num_steps):
    x = np.array([-17, 0, 17, 0])
    y = np.array([0, -17, 0, 17])
    left_x = np.array([-15, 0, 15, 0])
    left_y = np.array([0, -15, 0, 15])
    right_x = np.array([-19, 0, 19, 0])
    right_y = np.array([0, -19, 0, 19])

    # append the starting x,y coordinates
    x = np.r_[x, x[0]]
    y = np.r_[y, y[0]]
    left_x = np.r_[left_x, left_x[0]]
    left_y = np.r_[left_y, left_y[0]]
    right_x = np.r_[right_x, right_x[0]]
    right_y = np.r_[right_y, right_y[0]]

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x, y], s=0, per=True)
    tckl, u = interpolate.splprep([left_x, left_y], s=0, per=True)
    tckr, u = interpolate.splprep([right_x, right_y], s=0, per=True)

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)
    xl, yl = interpolate.splev(np.linspace(0, 1, num_steps), tckl)
    xr, yr = interpolate.splev(np.linspace(0, 1, num_steps), tckr)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, 'or')
    # ax.plot(xl, yl, '--g')
    # ax.plot(xr, yr, '--g')
    # plt.show()

    return [xi, yi, xl, yl, xr, yr]


def generate_racing_track(num_steps):
    # x = np.array([0, 4, 8, 12, 17, 20, 25, 29, 30, 32, 37, 40, 43, 45, 47, 56, 46, 35, 31, 25, 20, 15, 10, 0, -6])
    # y = np.array([0, 1, 5, 10, 7, 1, 9, 11, 8, 4, 0, -3, -9, -5, 0, 13, 27, 19, 29, 22, 29, 22, 17, 15, 8])
    # xl = np.array([0, 4, 8, 12, 17, 20, 25, 29, 30, 32, 37, 40, 43, 45, 47, 54, 46, 35, 31, 25, 20, 15, 10, 0, -4])
    # xr = np.array([0, 4, 8, 12, 17, 20, 25, 29, 30, 32, 37, 40, 43, 45, 47, 58, 46, 35, 31, 25, 20, 15, 10, 0, -8])
    x  = np.array([0, 8, 17, 25, 30, 37, 43, 47, 50, 46, 31, 20, 10, -6])
    y  = np.array([0, 5,  7,  9,  8,  0, -5,  0, 10, 27, 29, 29, 17, 15])
    xl = np.array([0, 8, 17, 25, 30, 37, 43, 47, 48, 46, 31, 20, 10, -4])
    xr = np.array([0, 8, 17, 25, 30, 37, 43, 47, 52, 46, 31, 20, 10, -8])
    # x = np.array([1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 25, 27, 29, 31, 33, 35, 37, 39])
    # y = np.array([0, 0, 0, 0, 0,  0,  0,  0,  0,  0, 10, 10, 10, 10, 10, 10, 10, 10])
    # xl = x
    # xr = x

    x_max = np.max(x)
    swp = False
    yl = []
    yr = []
    for i in range(len(x)):
        if x[i] >= x_max:
            swp = True
        if swp:
            yl.append(y[i] - param.track_width) # / 2)
            yr.append(y[i] + param.track_width) # / 2)
        else:
            yl.append(y[i] + param.track_width) # / 2)
            yr.append(y[i] - param.track_width) # / 2)

    # append the starting x,y coordinates
    x = np.r_[x, x[0]]
    y = np.r_[y, y[0]]
    xl = np.r_[xl, xl[0]]
    yl = np.r_[yl, yl[0]]
    xr = np.r_[xr, xr[0]]
    yr = np.r_[yr, yr[0]]

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x, y], s=3, per=True)
    tckl, u = interpolate.splprep([xl, yl], s=3, per=True)
    tckr, u = interpolate.splprep([xr, yr], s=3, per=True)

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)
    xl, yl = interpolate.splev(np.linspace(0, 1, num_steps), tckl)
    xr, yr = interpolate.splev(np.linspace(0, 1, num_steps), tckr)

    # plot the track
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, 'or')
    # ax.plot(xl, yl, 'ob')
    # ax.plot(xr, yr, 'oy')
    # plt.show()

    return [xi, yi, xl, yl, xr, yr]
