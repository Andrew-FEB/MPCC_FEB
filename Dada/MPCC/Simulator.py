import sys

import matplotlib.pyplot as plt
import numpy as np
import opengen as og

import CodeGenerator as cg


# Author: Darina Abaffyová
# Created: 13/02/2020
# Last updated: 29/02/2020


def simulate(x_state_0, ref, simulation_steps):
    # Simulation
    # -------------------------------------
    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("mpcc_python_build_1/mpcc_optimizer")
    # Start the TCP server
    mng.start()
    # Run simulations
    state_sequence = x_state_0
    input_sequence = []
    x = x_state_0
    f = cg.tire_forces(x_state_0, [0, 0])
    for k in range(simulation_steps):
        solver_status = mng.call(np.concatenate((x, ref)))
        try:
            us = solver_status['solution']
            u1 = us[0]
            u2 = us[1]
            forces = cg.tire_forces_dt(f, x, [u1, u2], cg.Ts)
            x_next = cg.dynamic_model(x, [u1, u2], forces, cg.Ts)
            state_sequence = np.concatenate((state_sequence, x_next))
            input_sequence += [u1, u2]
            x = x_next
        except AttributeError:
            print('Failed after ' + str(state_sequence.__len__() / cg.nx) + 'simulation steps\n'
                  + 'Error[' + str(solver_status['code']) + ']: ' + solver_status['message'])
            mng.kill()
            sys.exit(0)

        print('Loop [' + str(k) + ']: ' + str(solver_status['solve_time_ms']))

    mng.kill()

    return [input_sequence, state_sequence]


def plot_simulation(simulation_steps, input_sequence, state_sequence):
    t = np.arange(0, cg.Ts * simulation_steps, cg.Ts)
    plt.plot(t, input_sequence[0:cg.nu * simulation_steps:cg.nu], '-', label="Duty cycle")
    plt.plot(t, input_sequence[1:cg.nu * simulation_steps:cg.nu], '-', label="Front steering angle")
    plt.grid()
    plt.ylabel('Input')
    plt.xlabel('Time')
    plt.title('INPUT SEQUENCE')
    plt.legend(bbox_to_anchor=(0.7, 0.85), loc='best', borderaxespad=0.)
    plt.show()
    plt.plot(t, state_sequence[0:cg.nx * simulation_steps:cg.nx], '-', label="x")
    plt.plot(t, state_sequence[1:cg.nx * simulation_steps:cg.nx], '-', label="y")
    plt.plot(t, state_sequence[2:cg.nx * simulation_steps:cg.nx], '-', label="phi")
    plt.plot(t, state_sequence[3:cg.nx * simulation_steps:cg.nx], '-', label="v_x")
    plt.plot(t, state_sequence[4:cg.nx * simulation_steps:cg.nx], '-', label="v_y")
    plt.plot(t, state_sequence[5:cg.nx * simulation_steps:cg.nx], '-', label="omega")
    plt.grid()
    plt.ylabel('States')
    plt.xlabel('Time')
    plt.title('STATE SEQUENCE')
    plt.legend(bbox_to_anchor=(0.7, 0.85), loc='best', borderaxespad=0.)
    plt.show()
