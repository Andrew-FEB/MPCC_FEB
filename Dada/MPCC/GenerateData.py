import CodeGenerator as cg
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt


# import rosbag

# Author: Darina Abaffyová
# Created: 04/03/2020
# Last updated: 23/03/2020


def generate_reference_state(start_state, control, num_steps):
    s = np.concatenate((start_state, [0] * cg.nx))
    for i in range(0, num_steps):
        tf = cg.tire_forces(s, control)
        s = cg.dynamic_model_rk(s, control, tf, cg.Ts, False)

    return s[0:6]


def generate_track(num_steps):
    x_plus = np.array([7, -4.5, 11, 25.5, 16.5, 21.5])
    y_plus = np.array([10.5, -1.5, -1.5, 6.5, 6, 13])

    x_minus = np.array([7, -1.5, 11, 21, 13.5, 18.5])
    y_minus = np.array([7.5, 1, 1, 4.5, 4, 11.5])

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
    x = np.array([0, 8, 12, 16, 20, 30, 40])
    y = np.array([0, 1.5, 2.5, 3, 1, 0.5, 0])
    upper = np.array([1.5, 3, 4, 4.5, 2.5, 2, 1.5])
    lower = np.array([-1.5, 0, 1, 1.5, -0.5, -1, -1.5])

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x, y], s=0, per=False)
    tcku, u = interpolate.splprep([x, upper], s=0, per=False)
    tckl, u = interpolate.splprep([x, lower], s=0, per=False)

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)
    xu, yu = interpolate.splev(np.linspace(0, 1, num_steps), tcku)
    xl, yl = interpolate.splev(np.linspace(0, 1, num_steps), tckl)

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
    y = np.arange(0, 3, 3 / num_steps)
    x = np.arange(0, 17, 17 / num_steps)

    upper = np.arange(1.5, 4.5, 3 / num_steps)
    lower = np.arange(-1.5, 1.5, 3 / num_steps)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(x, y, 'or')
    # ax.plot(x, upper, '--g')
    # ax.plot(x, lower, '--g')
    # plt.show()

    return [x, y, upper, lower]

# def read_data():
#     data = []
#     bag = rosbag.Bag('test.bag')
#     for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
#         print(msg)
#     bag.close()
#     return data
