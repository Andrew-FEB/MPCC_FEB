import time

import datagenerator as gd
import simulator as sim
import visualisator as vis
import codegenerator as cg

# Author: Darina Abaffyová
# Created: 24/02/2020
# Last updated: 30/05/2020

track_points = 1000
sim_steps = 300

# Weights in cost function
track_error_weight = [0, 1, 0]  # contouring, tracking, velocity
in_weight = [1e-4, 1e-4]  # duty cycle, steering angle
in_change_weight = [0.01, 0.001]  # change of duty cycle, change of steering angle

print("Generating code")
start_time = time.time()
# cg.generate_code_kinematic(track_error_weight, in_weight, in_change_weight, 'p')
print('Code generated in ' + str(time.time() - start_time) + ' s')

print("Running simulation")
[track_x, track_y, upper, lower] = gd.generate_racing_track(track_points)
state_seq, ref_seq, bound_seq, cost_seq, first_input_seq, control_seq, sim_steps = sim.simulate(track_x, track_y,
                                                                                                upper, lower, sim_steps)
vis.plot_dynamic(track_x, track_y, upper, lower, state_seq, ref_seq, bound_seq, cost_seq, control_seq)
