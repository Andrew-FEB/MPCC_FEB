import CodeGenerator as cg
import Simulator as sim
import GenerateData as gd
import time
import numpy as np

# Author: Darina Abaffyová
# Created: 24/02/2020
# Last updated: 09/03/2020


simulation_steps = 10

state_error_weight = [1000, 1000, 0, 0, 0, 0]
in_weight = [0, 0]
in_change_weight = [0, 0]

print("Generating code")
st = time.time()
cg.generate_code(state_error_weight, in_weight, in_change_weight)
print('Code generated in ' + str(time.time() - st) + ' s')

print("Running simulation")
if simulation_steps > 1:
    [track_x, track_y] = gd.generate_track(simulation_steps)
    [in_seq, state_seq, state_ref] = sim.simulate(track_x, track_y, simulation_steps)
    sim.plot_simulation(simulation_steps, in_seq, state_seq, state_ref)
    sim.plot_track(state_ref, state_seq)
else:
    # [track_x, track_y] = gd.generate_track(100)
    # u = [cg.d_max / 3, cg.delta_max / 3]
    # state_0 = np.concatenate(([track_x[0], track_y[0]], [np.arctan2(track_y[0]-track_y[99], track_x[0]-track_x[99]),
    #                           45, 25, 2]))
    # ref_rest = [45, 25, 2]
    # [x, y] = [track_x[19], track_y[19]]
    # [x_prev, y_prev] = [track_x[18], track_y[18]]
    # phi = np.arctan2(y - y_prev, x - x_prev)
    # state_ref = np.concatenate(([x, y, phi], ref_rest))

    state_0 = [-3, 0, 0, 0, 0, 0]
    state_ref = [0.723, -2.915, -1.259, 15.774, 3.103, 0.736]

    [in_seq, state_seq] = sim.simulate_one_step(state_0, state_ref)
    sim.plot_simulation(cg.N, in_seq, state_seq, state_ref)

s = state_seq.size
print("Simulation finished with:\nstate_0 = " + str([state_seq[:6]]) + "\nstate_ref = " + str([state_seq[s - 6:s]]))
