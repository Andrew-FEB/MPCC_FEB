import CodeGenerator as cg
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
# import rosbag

# Author: Darina Abaffyová
# Created: 04/03/2020
# Last updated: 15/03/2020


def generate_reference_state(start_state, control, num_steps):
    s = np.concatenate((start_state, [0] * cg.nx))
    for i in range(0, num_steps):
        tf = cg.tire_forces(s, control)
        s = cg.dynamic_model_rk(s, control, tf, cg.Ts, False)

    return s[0:6]


def generate_track(num_steps):
    # https://stackoverflow.com/questions/33962717/interpolating-a-closed-curve-using-scipy
    # x = np.array([24, 25, 25, -3])
    # y = np.array([4, 12, 13, 0])
    # x = np.array([20, 7, -3, 11, 23, 15])
    # y = np.array([12, 9, 0, 0, 5.5, 5])
    # x = np.array([2.4, 2.5, 2.5, -0.3])
    # y = np.array([0.4, 1.2, 1.3, 0])

    # x_plus = np.array([7, -4.5, 11, 25.5, 16.5, 21.5])
    # y_plus = np.array([10.5, -1.5, -1.5, 6.5, 6, 13])
    #
    # x_minus = np.array([7, -1.5, 11, 21, 13.5, 18.5])
    # y_minus = np.array([7.5, 1, 1, 4.5, 4, 11.5])

    x_plus = np.array([0.7, -0.45, 1.1, 2.55, 1.65, 2.15])
    y_plus = np.array([1.05, -0.15, -0.15, 0.65, 0.6, 1.3])

    x_minus = np.array([0.7, -0.15, 1.1, 2.1, 1.35, 1.85])
    y_minus = np.array([0.75, 0.1, 0.1, 0.45, 0.4, 1.15])

    x = [0] * 6
    y = [0] * 6
    for i in range(6):
        x[i] = x_plus[i] - (x_plus[i] - x_minus[i])/2
        y[i] = y_plus[i] - (y_plus[i] - y_minus[i])/2

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


# def read_data():
#     data = []
#     bag = rosbag.Bag('test.bag')
#     for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
#         print(msg)
#     bag.close()
#     return data
