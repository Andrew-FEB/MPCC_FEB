import time
import codegenerator as cg
import datagenerator as gd
import simulator as sim
import visualisator as vis

# Author: Darina Abaffyová
# Created: 24/02/2020
# Last updated: 03/04/2020

track_points = 1000
simulation_steps = 100

# Weights in cost function
track_error_weight = [0, 1, 0]  # contouring, tracking, velocity
in_weight = [0, 0]  # duty cycle, steering angle
in_change_weight = [0, 0]  # change of duty cycle, change of steering angle
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
state_seq, reference_seq, nearest_seq, cost_seq, control_inputs_seq = sim.simulate(track_x, track_y, upper, lower, simulation_steps)
vis.plot_dynamic(track_x, track_y, upper, lower, state_seq, reference_seq, nearest_seq, cost_seq, control_inputs_seq)