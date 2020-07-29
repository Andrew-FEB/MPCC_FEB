import time

import datagenerator as dg
import simulator as sim
import visualisator as vis
import codegenerator as cg

# Author: Darina Abaffyová
# Created: 24/02/2020

track_points = 10000
sim_steps = 500

print("Generating code")
start_time = time.time()
# cg.generate_code('p')
print('Code generated in ' + str(time.time() - start_time) + ' s')

# print("Running simulation")
# [track_x, track_y, left_x, left_y, right_x, right_y] = dg.generate_racing_track(track_points)
# state_seq, ref_seq, bound_seq, cost_seq, solve_time_seq, exit_status_seq, first_input_seq, control_seq, sim_steps \
#     = sim.simulate(track_x, track_y, left_x, left_y, right_x, right_y, sim_steps)
# vis.plot_solve_time(solve_time_seq)
# vis.plot_dynamic(track_x, track_y, left_x, left_y, right_x, right_y, state_seq, ref_seq, bound_seq, cost_seq,
#                  control_seq, exit_status_seq)