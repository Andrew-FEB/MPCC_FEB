import time

import datagenerator as gd
import simulator as sim
import visualisator as vis
import codegenerator as cg

# Author: Darina Abaffyová
# Created: 24/02/2020

track_points = 1500  # 25  #
sim_steps = 500

print("Generating code")
start_time = time.time()
cg.generate_code('c')
print('Code generated in ' + str(time.time() - start_time) + ' s')

# print("Running simulation")
# [track_x, track_y, upper, lower] = gd.generate_racing_track(track_points)
# state_seq, ref_seq, bound_seq, cost_seq, solve_time_seq, first_input_seq, control_seq, sim_steps \
#     = sim.simulate(track_x, track_y, upper, lower, sim_steps)
# vis.plot_solve_time(solve_time_seq)
# vis.plot_dynamic(track_x, track_y, upper, lower, state_seq, ref_seq, bound_seq, cost_seq, control_seq)
