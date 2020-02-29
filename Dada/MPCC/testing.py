import CodeGenerator as cg
import Simulator as sim

# Author: Darina Abaffyová
# Created: 24/02/2020
# Last updated: 29/02/2020


simulation_steps = 100
state_0 = [0, 0, 0.785, 100, 100, 30]  #
ref = [1, 1, 0.785, 100, 100, 30]  # [1, 1, 0.785, 0.1, 0.1, 0.3]  #
state_error_weight = 150
d_change_weight = 9
delta_change_weight = 13

# print("Generating code")
# cg.generate_code(state_error_weight, d_change_weight, delta_change_weight)
print("Running simulation")
[in_seq, state_seq] = sim.simulate(state_0, ref, simulation_steps)
print("Plotting")
sim.plot_simulation(simulation_steps, in_seq, state_seq)
