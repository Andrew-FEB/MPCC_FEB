import time

import CodeGenerator as cg
import GenerateData as gd
import Simulator as sim

# Author: Darina Abaffyová
# Created: 24/02/2020
# Last updated: 23/03/2020

simulation_steps = 1000

contouring_error_weight = [100, 10]
# in_weight = [0, 0]
# in_change_weight = [0, 0]

print("Generating code")
st = time.time()
# cg.generate_code(contouring_error_weight)  # , in_weight, in_change_weight)
print('Code generated in ' + str(time.time() - st) + ' s')

print("Running simulation")
if simulation_steps > 1:
    [track_x, track_y, upper, lower] = gd.generate_linear_track(simulation_steps)
    # [track_x, track_y] = gd.generate_track(simulation_steps)
    [in_seq, state_seq, state_ref, nearest_seq, simulation_steps] = sim.simulate(track_x, track_y, simulation_steps)
    sim.plot_simulation(simulation_steps, in_seq, state_seq, state_ref)
    sim.plot_track(track_x, track_y, upper, lower, state_ref, state_seq)
    sim.plot_nearest(track_x, track_y, nearest_seq, state_seq)
else:
    state_0 = [0.0, 3.0, 1.5707963267948966, 10, 7, 0.5]
    state_ref = [1.104, 2.44800000000002, -0.4636476090007913, 15, 5, 0.7]

    [in_seq, state_seq] = sim.simulate_one_step(state_0, state_ref)
    ref_sequence = []
    for i in range(len(state_seq)):
        ref_sequence.append(tuple(state_ref))
    sim.plot_simulation(cg.N, in_seq, state_seq, ref_sequence)
