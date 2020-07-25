import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate


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
    x = np.array([0, 4, 8, 12, 17, 20, 25, 29, 30, 32, 37, 40, 43, 45, 47, 56, 46, 35, 31, 25, 20, 15, 10, 0, -6])
    y = np.array([0, 1, 5, 10, 7, 1, 9, 11, 8, 4, 0, -3, -9, -5, 0, 13, 27, 19, 29, 22, 29, 22, 17, 15, 8])
    xl = np.array([0, 4, 8, 12, 17, 20, 25, 29, 30, 32, 37, 40, 43, 45, 47, 54, 46, 35, 31, 25, 20, 15, 10, 0, -4])
    xr = np.array([0, 4, 8, 12, 17, 20, 25, 29, 30, 32, 37, 40, 43, 45, 47, 58, 46, 35, 31, 25, 20, 15, 10, 0, -8])

    x_max = np.max(x)
    swp = False
    yl = []
    yr = []
    for i in range(len(x)):
        if x[i] >= x_max:
            swp = True
        if swp:
            yl.append(y[i] - 3.5)  # param.track_width/2)
            yr.append(y[i] + 3.5)  # param.track_width/2)
        else:
            yl.append(y[i] + 3.5)  # param.track_width/2)
            yr.append(y[i] - 3.5)  # param.track_width/2)

    # append the starting x,y coordinates
    x = np.r_[x, x[0]]
    y = np.r_[y, y[0]]
    xl = np.r_[xl, xl[0]]
    yl = np.r_[yl, yl[0]]
    xr = np.r_[xr, xr[0]]
    yr = np.r_[yr, yr[0]]

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x, y], s=0, per=False)
    tckl, u = interpolate.splprep([xl, yl], s=0, per=False)
    tckr, u = interpolate.splprep([xr, yr], s=0, per=False)

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)
    xl, yl = interpolate.splev(np.linspace(0, 1, num_steps), tckl)
    xr, yr = interpolate.splev(np.linspace(0, 1, num_steps), tckr)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, 'or')
    # ax.plot(xl, yl, 'og')
    # ax.plot(xr, yr, 'ob')
    # plt.show()

    return [xi, yi, xl, yl, xr, yr]


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


# TODO - this is not working
def read_rosbag_track():
    import rosbag
    bag = rosbag.Bag('KartingGenk.bag')
    for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
        print(msg)
    bag.close()
