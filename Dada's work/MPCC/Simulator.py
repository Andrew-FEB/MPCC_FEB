import CodeGenerator as cg
import numpy as np
import matplotlib.pyplot as plt
import opengen as og
import time

# Author: Darina Abaffyová
# Created: 13/02/2020
# Last updated: 17/02/2020

# Simulation
# -------------------------------------
# Create a TCP connection manager
mng = og.tcp.OptimizerTcpManager("mpcc_python_build/mpcc_optimizer")

# Start the TCP server
mng.start()

# Run simulations
x_state_0 = [0, 0, 0, 0]  # [1.0, 1.0, 0.785, 0.1]
simulation_steps = 500

state_sequence = x_state_0
input_sequence = []

st = time.time()

x = x_state_0
for k in range(simulation_steps):
    # start_time = time.time()
    solver_status = mng.call(x)
    try:
        us = solver_status['solution']
        u1 = us[0]
        u2 = us[1]
        x_next = cg.vehicle_dynamics_dt(x, [u1, u2], cg.Ts)
        state_sequence = np.concatenate((state_sequence, x_next))
        input_sequence += [u1, u2]
        x = x_next
    except AttributeError:
        print('Failed after ' + str(state_sequence.__len__() / 4) + 'simulation steps')
        simulation_steps = int(state_sequence.__len__() / 4) -1
        # exit(1)
        break
    # print('Loop time = ' + str(time.time() - start_time) + 's')

# solver_status = mng.call(x_state_0)
# us = solver_status['solution']
# x = x_state_0
# for i in range(0, 80, 2):
#     x = cg.vehicle_dynamics_dt(x, [us[i], us[i+1]], cg.Ts)
#     state_sequence = np.concatenate((state_sequence, x))
#     input_sequence += [us[i], us[i+1]]

print('Looping: ' + str(time.time() - st))

# Thanks TCP server; we won't be needing you any more
mng.kill()

time = np.arange(0, cg.Ts * simulation_steps, cg.Ts)

plt.plot(time, input_sequence[0:2 * simulation_steps:2], '-', label="Front Steering Angle")
plt.plot(time, input_sequence[1:2 * simulation_steps:2], '-', label="Acceleration")
plt.grid()
plt.ylabel('Input')
plt.xlabel('Time')
plt.title('[X, Y, PSI, V]: state_0 = ' + str(x_state_0) + ', state_ref = ' + str(cg.x_ref)
          + '\nstate error weight = ' + str(cg.state_error_weight)
          + ', input change weight = ' + str(cg.input_change_weight))
plt.legend(bbox_to_anchor=(0.7, 0.85), loc='best', borderaxespad=0.)
plt.show()

plt.plot(time, state_sequence[0:4 * simulation_steps:4], '-', label="x")
plt.plot(time, state_sequence[1:4 * simulation_steps:4], '-', label="y")
plt.plot(time, state_sequence[2:4 * simulation_steps:4], '-', label="psi")
plt.plot(time, state_sequence[3:4 * simulation_steps:4], '-', label="v")
plt.grid()
plt.ylabel('States')
plt.xlabel('Time')
plt.title('[X, Y, PSI, V]: state_0 = ' + str(x_state_0) + ', state_ref = ' + str(cg.x_ref)
          + '\nstate error weight = ' + str(cg.state_error_weight)
          + ', input change weight = ' + str(cg.input_change_weight))
# + '\n' + str(simulation_steps) + ' simulation steps')
plt.legend(bbox_to_anchor=(0.7, 0.85), loc='best', borderaxespad=0.)
plt.show()
