import CodeGenerator as cg
import Simulator as sim

# Author: Darina Abaffyová
# Created: 24/02/2020
# Last updated: 25/02/2020


simulation_steps = 13
state_0 = [0, 0, 0, 0, 0, 0]
ref = [0] * cg.nx
state_error_weight = 70
input_change_weight = 0

# print("Generating code")
# cg.generate_code(ref, state_error_weight, input_change_weight)
print("Running simulation")
[in_seq, state_seq] = sim.simulate(state_0, simulation_steps)
print("Plotting")
sim.plot_simulation(simulation_steps, in_seq, state_seq)
