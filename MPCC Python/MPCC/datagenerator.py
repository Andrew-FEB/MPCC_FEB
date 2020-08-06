import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate

import parameters as param


# Author: Darina Abaffyová
# Created: 04/03/2020

def generate_linear_track(num_steps):
    len_x = 10
    len_y = 70
    x = np.arange(len_x, 0, -len_x / num_steps)
    y = np.arange(len_y, 0, -len_y / num_steps)  # [0] * len(x)

    left = np.arange(param.track_width + len_y, param.track_width, -len_y / num_steps)
    right = np.arange(-param.track_width + len_y, -param.track_width, -len_y / num_steps)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(x, y, 'or')
    # ax.plot(x, left, '--y')
    # ax.plot(x, right, '--b')
    # plt.show()

    return [x, y, x, left, x, right]

    # y = np.arange(0, 40, 40 / num_steps)
    # x = [0] * len(y)
    #
    # left = [-param.track_width/2] * len(x)
    # right = [param.track_width/2] * len(x)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(x, y, 'or')
    # ax.plot(left, y, '--g')
    # ax.plot(right, y, '--g')
    # plt.show()

    # return [x, y, left, y, right, y]


def generate_circular_track(num_steps):
    radius_x = 50
    radius_y = 20
    width = param.track_width
    x = np.array([-radius_x, 0, radius_x, 0])
    y = np.array([0, -radius_y, 0, radius_y])
    left_x = np.array([-radius_x + width, 0, radius_x - width, 0])
    left_y = np.array([0, -radius_y + width, 0, radius_y - width])
    right_x = np.array([-radius_x - width, 0, radius_x + width, 0])
    right_y = np.array([0, -radius_y - width, 0, radius_y + width])
    plus = 30
    x = [x + plus for x in x]
    left_x = [xl + plus for xl in left_x]
    right_x = [xr + plus for xr in right_x]
    y = [y + plus for y in y]
    left_y = [yl + plus for yl in left_y]
    right_y = [yr + plus for yr in right_y]

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

    # for xt, yt in zip(xr, yr):
    #     print("(x,y) = (" + str(round(xt,2)) + ", " + str(round(yt,2)) + ")")

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, '.r')
    # ax.plot(x, y, 'xr', markersize=15)
    # ax.plot(xl, yl, '.y')
    # ax.plot(xr, yr, '.b')
    # plt.show()

    return [xi, yi, xl, yl, xr, yr]


def generate_racing_track(num_steps):
    # num_steps = 80
    # x = np.array([0, 4, 8, 12, 17, 20, 25, 29, 30, 32, 37, 40, 43, 45, 47, 56, 46, 35, 31, 25, 20, 15, 10,  0, -6])
    # y = np.array([0, 1, 5, 10,  7,  1,  9, 11,  8,  4,  0, -3, -9, -5,  0, 13, 27, 19, 29, 22, 29, 22, 17, 15, 8])
    # xl = np.array([0, 4, 8, 12, 17, 20, 25, 29, 30, 32, 37, 40, 43, 45, 47, 54, 46, 35, 31, 25, 20, 15, 10, 0, -4])
    # xr = np.array([0, 4, 8, 12, 17, 20, 25, 29, 30, 32, 37, 40, 43, 45, 47, 58, 46, 35, 31, 25, 20, 15, 10, 0, -8])
    # x = np.array([0,  10, 19, 27, 32, 39, 45, 60, 48, 33, 17, 12, -4])
    # y = np.array([10, -7, -3,  2,  5,  0, -9, 10, 14, 27, 19, 29, 18])
    # xl = np.array([1.5, 10, 19, 27, 32, 40, 46, 58, 48, 33, 17, 11, -1])
    # xr = np.array([-1.5, 10, 19, 27, 32, 38, 44, 62, 48, 33, 17, 13, -7])
    x = np.array([0,   10, 19, 27,  30,  34.37, 42.59, 48.03, 50, 48.03, 42.59, 39, 33, 19, 12,  5.63, -2.59,  -8.03, -10,  -8.03,  -2.59])
    y = np.array([9,  -7,  3, -5, -10, -13.75, -9.63, -4.95,  0,  4.95,  9.63, 16, 27, 19, 29, 33.75, 29.63,  24.95,  20,  15.05,  10.37])
    xl = np.array([1,  10, 19, 27,  30,  32.31, 40.04, 45.15, 47, 45.15, 40.04, 38, 33, 19, 12,  7.69, -0.04,  -5.15,  -7,  -5.15,  -0.04])
    xr = np.array([-1, 10, 19, 27,  30,  36.44, 45.15, 50.91, 53, 50.91, 45.15, 40, 33, 19, 12,  3.56, -5.15, -10.91, -13, -10.91,  -5.15])
    # x = np.array([1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 25, 27, 29, 31, 33, 35, 37, 39])
    # y = np.array([0, 0, 0, 0, 0,  0,  0,  0,  0,  0, 10, 10, 10, 10, 10, 10, 10, 10])
    # xl = x
    # xr = x
    # x  = np.array([0, 14, 23, 40, 48, 59, 66, 75, 80, 72, 50, 32, 20, -8, -19])
    # y  = np.array([0,  9,  3, 14, 12,  0, -6,  0, 20, 42, 46, 42, 27, 15,   5])
    # xl = np.array([0, 14, 23, 40, 48, 59, 66, 73, 78, 71, 50, 33, 20, -8, -15])
    # xr = np.array([0, 14, 23, 40, 48, 59, 66, 77, 82, 73, 50, 31, 20, -8, -22])
    # x = np.array([0, 4, 8, 14, 19, 27, 30, 33, 37, 40, 43, 47, 51, 55, 59])
    # y = np.array([0, 1, 5, 10,  7,  5,  9, 11,  8,  4,  0, -3, -9, -5,  0])
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
            yl.append(y[i] - param.track_width)
            yr.append(y[i] + param.track_width)
        else:
            yl.append(y[i] + param.track_width)
            yr.append(y[i] - param.track_width)

    yl[5:11] = [-10, -8.19, -4.21, 0, 4.21, 8.19]
    yl[15:21] = [31.69, 28.19, 24.21, 20, 15.79, 11.81]
    yr[5:11] = [-15.81, -11.07, -5.7, 0, 5.7, 11.07]
    yr[15:21] = [35.81, 31.07, 25.7, 20, 14.3, 8.93]

    multi = 1.5
    x = [x * multi for x in x]
    xl = [xl * multi for xl in xl]
    xr = [xr * multi for xr in xr]
    y = [y * multi for y in y]
    yl = [yl * multi for yl in yl]
    yr = [yr * multi for yr in yr]

    # append the starting x,y coordinates
    x = np.r_[x, x[0]]
    y = np.r_[y, y[0]]
    xl = np.r_[xl, xl[0]]
    yl = np.r_[yl, yl[0]]
    xr = np.r_[xr, xr[0]]
    yr = np.r_[yr, yr[0]]

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x, y], s=0, per=True)
    tckl, u = interpolate.splprep([xl, yl], s=0, per=True)
    tckr, u = interpolate.splprep([xr, yr], s=0, per=True)

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)
    xl, yl = interpolate.splev(np.linspace(0, 1, num_steps), tckl)
    xr, yr = interpolate.splev(np.linspace(0, 1, num_steps), tckr)

    # Calculate length and width of the track
    x_prev = xi[0]
    y_prev = yi[0]
    dist = 0
    for xt, yt in zip(xi, yi):
        dist += round(np.sqrt((xt - x_prev) ** 2 + (yt - y_prev) ** 2), 3)
        x_prev = xt
        y_prev = yt
    print("Length of the track is: " + str(dist))

    # x_prev = xl[0]
    # y_prev = yl[0]
    # for xt, yt in zip(xl, yl):
    #     print("Distance between LEFT cones = " + str(np.sqrt((xt - x_prev) ** 2 + (yt - y_prev) ** 2)))
    #     x_prev = xt
    #     y_prev = yt
    #
    # x_prev = xr[0]
    # y_prev = yr[0]
    # for xt, yt in zip(xr, yr):
    #     print("Distance between RIGHT cones = " + str(np.sqrt((xt - x_prev) ** 2 + (yt - y_prev) ** 2)))
    #     x_prev = xt
    #     y_prev = yt

    # plot the track
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, '-r', markersize=1, label="Reference line")
    # ax.plot(x, y, 'xr', markersize=15)
    # ax.plot(xl, yl, '.b', markersize=1, label="Track boundaries")
    # ax.plot(xr, yr, '.y', markersize=1)
    # plt.grid()
    # plt.legend()
    # plt.show()

    # Print to be used in C++ with BE
    # print("//LEFT")
    # for xlt, ylt in zip(xl, yl):
    #     # print("track->addCone(" + str(round(xlt,2)) + ", " + str(round(ylt,2)) + ", BoundPos::left);")
    #     print("{" + str(round(xlt,2)) + ", " + str(round(ylt,2)) + "},")
    #
    # print("\n//RIGHT")
    # for xrt, yrt in zip(xr, yr):
    #     # print("track->addCone(" + str(round(xrt,2)) + ", " + str(round(yrt,2)) + ", BoundPos::right);")
    #     print("{" + str(round(xrt, 2)) + ", " + str(round(yrt, 2)) + "},")

    return [xi, yi, xl, yl, xr, yr]
