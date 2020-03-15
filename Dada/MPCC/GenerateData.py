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
    x = np.array([25, 35, -3, 24, 5])
    y = np.array([12, 23, 0, 4, 5])
    # x = np.array([2.4, 2.5, 2.5, -0.3])
    # y = np.array([0.4, 1.2, 1.3, 0])

    # append the starting x,y coordinates
    x = np.r_[x, x[0]]
    y = np.r_[y, y[0]]

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck = interpolate.splprep([x, y], s=0, per=True)

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, num_steps), tck)

    # plot the result
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(xi, yi, 'or')
    # plt.show()

    return [xi, yi]


# def read_data():
#     data = []
#     bag = rosbag.Bag('test.bag')
#     for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
#         print(msg)
#     bag.close()
#     return data
