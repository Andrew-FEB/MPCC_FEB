import time

import CodeGenerator as cg
import GenerateData as gd
import Simulator as sim
import numpy as np
import matplotlib.pyplot as plt

# Author: Darina Abaffyová
# Created: 24/02/2020
# Last updated: 23/03/2020

simulation_steps = 300

track_error_weight = [1, 0.5, 1]
in_weight = [1, 1]
in_change_weight = [1, 1]

print("Generating code")
st = time.time()
# cg.generate_code(track_error_weight, in_weight, in_change_weight)
print('Code generated in ' + str(time.time() - st) + ' s')

print("Running simulation")
if simulation_steps > 1:
    # [track_x, track_y, upper, lower] = gd.generate_linear_track(simulation_steps)
    [track_x, track_y] = gd.generate_circular_track(simulation_steps)
    [in_seq, state_seq, state_ref, nearest_seq, simulation_steps] = sim.simulate(track_x, track_y, simulation_steps)
    sim.plot_simulation(simulation_steps, in_seq, state_seq, state_ref)
    sim.plot_track2(track_x, track_y, state_ref, state_seq)
    # sim.plot_track(track_x, track_y, upper, lower, state_ref, state_seq)
    # sim.plot_nearest(track_x, track_y, nearest_seq, state_seq)
else:
    state_0 = [0.0, 0.0, 0.0, 10, 7, 0.5]
    state_ref = [1.14, 0.57, 0.4636476090008061, 15, 5, 0.7]

    [in_seq, state_seq] = sim.simulate_one_step(state_0, state_ref)
    ref_sequence = []
    for i in range(len(state_seq)):
        ref_sequence.append(tuple(state_ref))
    sim.plot_simulation(cg.N, in_seq, state_seq, ref_sequence)


def test_cost_function(points):
    track_x, track_y, upper, lower = gd.generate_linear_track(simulation_steps)
    point1 = (0, 3)
    point2 = (-1, 3.5)
    point3 = (0, 0)
    ref_point = (5, 0.5)
    nearest_point_i1, dist1 = sim.find_closest_point_centreline(point1, track_x, track_y, 1.5, 1000, 0)
    nearest_point_i2, dist2 = sim.find_closest_point_centreline(point2, track_x, track_y, 1.5, 1000, 0)
    nearest_point_i3, dist3 = sim.find_closest_point_centreline(point3, track_x, track_y, 1.5, 1000, 0)

    nearest_point1 = [track_x[nearest_point_i1], track_y[nearest_point_i1]]
    nearest_point2 = [track_x[nearest_point_i2], track_y[nearest_point_i2]]
    nearest_point3 = [track_x[nearest_point_i3], track_y[nearest_point_i3]]

    cost1 = cost_function(point1, ref_point, nearest_point1, -0.5, track_error_weight)
    cost2 = cost_function(point2, ref_point, nearest_point2, -0.5, track_error_weight)
    cost3 = cost_function(point3, ref_point, nearest_point3, -0.5, track_error_weight)

    # plot the result
    fig, ax = plt.subplots(1, 1)
    ax.plot(track_x, track_y, 'or')
    ax.plot(track_x, upper, '--g')
    ax.plot(track_x, lower, '--g')
    ax.plot(point1[0], point1[1], 'xb')
    ax.plot(point2[0], point2[1], 'xb')
    ax.plot(point3[0], point3[1], 'xb')
    ax.plot(ref_point[0], ref_point[1], 'xk')
    plt.text(point1[0], point1[1], str([round(cost1, 3), round(dist1, 3)]), color="blue", fontsize=10)
    plt.text(point2[0], point2[1], str([round(cost2, 3), round(dist2, 3)]), color="blue", fontsize=10)
    plt.text(point3[0], point3[1], str([round(cost3, 3), round(dist3, 3)]), color="blue", fontsize=10)
    plt.text(ref_point[0], ref_point[1], "REF")
    plt.grid()
    plt.show()

    print(str([cost1, cost2, cost3]))


def cost_function(point, ref_point, nearest_point, slope, weight):
    # TODO - add remaining costs
    cf = 0

    x = point[0]
    y = point[1]
    x_ref = ref_point[0]
    y_ref = ref_point[1]
    x_nearest = nearest_point[0]
    y_nearest = nearest_point[1]

    # y = slope * x + y_intercept
    y_inter = y_nearest - slope * x_nearest

    # Line: ax + by + c = 0 -> slope * x - y + y_inter = 0
    # Point: (x1, y1) -> (x, y)
    # Distance = (| a*x1 + b*y1 + c |) / (sqrt(a*a + b*b))

    # Contouring Error
    cf += weight[0] * (abs((slope * x - y + y_inter)) / (np.sqrt(slope ** 2 + 1)))

    # Tracking Error
    cf += weight[1] * np.sqrt((x - x_ref) ** 2 + (y - y_ref) ** 2)

    return cf
