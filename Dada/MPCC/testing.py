import time

import CodeGenerator as cg
import GenerateData as gd
import Simulator as sim

# Author: Darina Abaffyová
# Created: 24/02/2020
# Last updated: 03/04/2020

track_points = 1000
simulation_steps = 100

# Weights in cost function
track_error_weight = [1, 1, 1]  # contouring, tracking, velocity
in_weight = [1e-4, 1e-4]  # duty cycle, steering angle
in_change_weight = [0.01, 0.1]  # change of duty cycle, change of steering angle
# track_error_weight = [0.1, 1000, 2]  # contouring, tracking, velocity
# in_weight = [1e-4, 1e-4]  # duty cycle, steering angle
# in_change_weight = [0.01, 1]  # change of duty cycle, change of steering angle

print("Generating code")
start_time = time.time()
# cg.generate_code(track_error_weight, in_weight, in_change_weight, 'p')
# cg.generate_code_warm_start(track_error_weight, in_weight, in_change_weight, 'p')
print('Code generated in ' + str(time.time() - start_time) + ' s')

print("Running simulation")
[track_x, track_y, upper, lower] = gd.generate_linear_track(track_points)
# [track_x, track_y] = gd.generate_track(track_points)
sim.simulate(track_x, track_y, upper, lower, simulation_steps)


def save_track_to_file(t_x, t_y, up, low):
    file = open("track.txt", "a+")
    file.write("")
    for i in range(0, len(t_x)):
        file.write("%d %d %d %d\n" % (t_x[i], t_y[i], up[i], low[i]))

    return 0
