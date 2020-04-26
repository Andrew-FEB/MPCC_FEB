import time

import CodeGenerator as cg
import GenerateData as gd
import Simulator as sim

# Author: Darina Abaffyová
# Created: 24/02/2020
# Last updated: 03/04/2020

track_points = 300
simulation_steps = 500

# Weights in cost function
track_error_weight = [1, 1.22, 0.35]  # contouring, tracking, velocity
in_weight = [0.1, 0.1]  # acceleration, steering angle
in_change_weight = [0.3, 0.1]  # change of acceleration, change of steering angle

print("Generating code")
start_time = time.time()
cg.generate_code(track_error_weight, in_weight, in_change_weight)
print('Code generated in ' + str(time.time() - start_time) + ' s')

print("Running simulation")
[track_x, track_y, upper, lower] = gd.generate_racing_track(track_points)

# [track_x, track_y] = gd.generate_circular_track(track_points)
[in_seq, state_seq, ref_seq, nearest_seq, simulation_steps] = sim.simulate(track_x, track_y,
                                                                           upper, lower, simulation_steps)
sim.plot_simulation(simulation_steps, in_seq, state_seq, ref_seq)
# sim.plot_track2(track_x, track_y, ref_seq, state_seq)
sim.plot_track(track_x, track_y, upper, lower, ref_seq, state_seq)
# sim.plot_nearest(track_x, track_y, nearest_seq, state_seq)
sim.plot_cost(state_seq, )


def save_track_to_file(track_x, track_y, upper, lower):
    file = open("track.txt", "a+")
    file.write("")
    for i in range(0, len(track_x)):
        file.write("%d %d %d %d\n" % (track_x[i], track_y[i], upper[i], lower[i]))

    return 0
