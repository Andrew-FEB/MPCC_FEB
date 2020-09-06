import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate

import parameters as param


# Author: Darina Abaffyová
# Created: 04/03/2020

def generate_linear_track(num_steps):
    len_x = 70
    len_y = 100
    x = np.arange(0, len_x, len_x / num_steps)
    y = np.arange(0, len_y, len_y / num_steps)

    left = np.arange(param.track_width, param.track_width + len_y, len_y / num_steps)
    right = np.arange(-param.track_width, -param.track_width + len_y, len_y / num_steps)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(x, y, 'or')
    # ax.plot(x, left, '--y')
    # ax.plot(x, right, '--b')
    # plt.show()

    return [x, y, x, left, x, right]


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
    # Move by centre to point (plus, plus)
    # plus = 30
    # x = [x + plus for x in x]
    # left_x = [xl + plus for xl in left_x]
    # right_x = [xr + plus for xr in right_x]
    # y = [y + plus for y in y]
    # left_y = [yl + plus for yl in left_y]
    # right_y = [yr + plus for yr in right_y]

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

    # evaluate the spline fits for num_steps evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)
    xl, yl = interpolate.splev(np.linspace(0, 1, num_steps), tckl)
    xr, yr = interpolate.splev(np.linspace(0, 1, num_steps), tckr)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, '.r')
    # ax.plot(x, y, 'xr', markersize=15)
    # ax.plot(xl, yl, '.y')
    # ax.plot(xr, yr, '.b')
    # plt.show()

    return [xi, yi, xl, yl, xr, yr]


def generate_racing_track(num_steps):
    x = np.array([-2.59, 7.5, 15, 23, 30, 34.37, 42.59, 48.03, 50, 48.03, 42.59, 39, 33, 23, 12, 5.63, -2.59, -8.03, -10, -8.03])
    y = np.array([9.37, -5, 0, -2, -10, -13.75, -9.63, -4.95, 0, 4.95, 9.63, 16, 27, 19, 29, 32.75, 28.63, 23.95, 19, 14.05])
    xl = np.array([-0.04, 7.5, 15, 23, 30, 32.31, 40.04, 45.15, 47, 45.15, 40.04, 38, 33, 23, 12, 7.69, -0.04, -5.15, -7, -5.15])
    xr = np.array([-5.15, 7.5, 15, 23, 30, 36.44, 45.15, 50.91, 53, 50.91, 45.15, 40, 33, 23, 12, 3.56, -5.15, -10.91, -13, -10.91])

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

    yl[0] = 10.81
    yl[5:11] = [-10, -8.19, -4.21, 0, 4.21, 8.19]
    yl[15:20] = [30.69, 27.19, 23.21, 19, 14.79]
    yr[0] = 7.93
    yr[5:11] = [-15.81, -11.07, -5.7, 0, 5.7, 11.07]
    yr[15:20] = [34.81, 30.07, 24.7, 19, 13.3]

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

    # evaluate the spline fits for num_steps evenly spaced distance values
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

    # Find distances between boundary points
    # x_prev = xl[0]
    # y_prev = yl[0]
    # dist = [0]*len(xl)
    # i = 0
    # for xt, yt in zip(xl[1:len(xl)], yl[1:len(yl)]):
    #     dist[i] = np.sqrt((xt - x_prev) ** 2 + (yt - y_prev) ** 2)
    #     x_prev = xt
    #     y_prev = yt
    #     i += 1
    # print("Min dist = " + str(np.min(dist[0:len(dist)-1])))
    # print("Max dist = " + str(np.max(dist[0:len(dist)-1])))
    #
    # x_prev = xr[0]
    # y_prev = yr[0]
    # i = 0
    # for xt, yt in zip(xr[1:len(xr)], yr[1:len(yr)]):
    #     dist[i] = np.sqrt((xt - x_prev) ** 2 + (yt - y_prev) ** 2)
    #     x_prev = xt
    #     y_prev = yt
    #     i += 1
    # print("Min dist = " + str(np.min(dist[0:len(dist)-1])))
    # print("Max dist = " + str(np.max(dist[0:len(dist)-1])))

    # plot the track
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, '-r', markersize=2)
    # ax.plot(x, y, 'xr', markersize=15)
    # ax.plot(xl, yl, '.b')  #, markersize=2)
    # ax.plot(xr, yr, '.y')  #, markersize=2)
    # plt.grid()
    # plt.axis('equal')
    # plt.xlabel("X position")
    # plt.ylabel("Y position")
    # plt.show()

    # Print to be used in C++ with Boundary Estimation
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
