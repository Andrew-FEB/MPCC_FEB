import time

import CodeGenerator as cg
import GenerateData as gd
import Simulator as sim

# Author: Darina Abaffyová
# Created: 24/02/2020
# Last updated: 15/03/2020


simulation_steps = 1000

state_error_weight = [370, 370, 27, 5, 4, 2]
in_weight = [1, 1]
in_change_weight = [1, 1]

print("Generating code")
st = time.time()
# cg.generate_code(state_error_weight, in_weight, in_change_weight)
print('Code generated in ' + str(time.time() - st) + ' s')

print("Running simulation")
if simulation_steps > 1:
    [track_x, track_y] = gd.generate_track(simulation_steps)
    [in_seq, state_seq, state_ref, simulation_steps] = sim.simulate(track_x, track_y, simulation_steps)
    sim.plot_simulation(simulation_steps, in_seq, state_seq, state_ref)
    sim.plot_track(state_ref, state_seq)
else:
    # state_0 = [-2.999999999999999, -3.3306690738754696e-16, -1.5707963267948968, 0.5, 0.5, 0] state_ref = [
    # 1.40300355, -3.05668885, -0.6068417, 1, 1, 0.7]  # [0.72304031, -2.91475585, -0.66422473, 1.3, 1, 0.7]
    state_0 = [23, -2, 0, 1, 1, 0]
    state_ref = [23.723, -2.915, -1.259, 15.774, 3.103, 0.736]

    [in_seq, state_seq] = sim.simulate_one_step(state_0, state_ref)
    sim.plot_simulation(cg.N, in_seq, state_seq, state_ref)

s = state_seq.size
print("Simulation finished with:\nstate_0 = " + str([state_seq[:6]]) + "\nstate_end = " + str([state_seq[s - 6:s]]))
