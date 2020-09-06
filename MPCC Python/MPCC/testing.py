import time

import datagenerator as dg
import simulator as sim
import visualisator as vis
import codegenerator as cg
import parameters as param

# Author: Darina Abaffyová
# Created: 24/02/2020

track_points = 15000
sim_steps = 2500

print("Generating code")
start_time = time.time()
# cg.generate_code('p')
print('Code generated in ' + str(time.time() - start_time) + ' s')

print("Running simulation")
[track_x, track_y, left_x, left_y, right_x, right_y] = dg.generate_racing_track(track_points)
state_seq, ref_seq, bound_seq, cost_seq, solve_time_seq, exit_status_seq, first_input_seq, control_seq, sim_steps \
    = sim.simulate(track_x, track_y, left_x, left_y, right_x, right_y, sim_steps)

vis.plot_solve_time(solve_time_seq, exit_status_seq)
vis.plot_track(track_x, track_y, left_x, left_y, right_x, right_y, state_seq)
# vis.plot_dynamic(track_x, track_y, left_x, left_y, right_x, right_y, state_seq, ref_seq, bound_seq, cost_seq,
#                  control_seq, exit_status_seq)

# Save in a file
# TRACK
# fn = 'N' + str(param.N) + 'tew' + str(param.track_error_weight) + '.txt'
# open(fn, 'w').close()
# file2write = open(fn, 'a')
# for i in range(len(state_seq)):
#     file2write.write(str(state_seq[i].x) + ", " + str(state_seq[i].y) + "\n")
# file2write.close()
# # TIMES
# fn = 'times_N' + str(param.N) + 'tew' + str(param.track_error_weight) + '.txt'
# open(fn, 'w').close()
# file2write = open(fn, 'a')
# for st in solve_time_seq:
#     file2write.write(str(st) + "\n")
# file2write.close()

# vis.plot_times_from_files()
# vis.plot_tracks_from_files(track_x, track_y, left_x, left_y, right_x, right_y)
# vis.plot_bempcc_times()
