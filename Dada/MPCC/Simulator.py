import sys

import matplotlib.pyplot as plt
import numpy as np
import opengen as og

import CodeGenerator as cg


# Author: Darina Abaffyová
# Created: 13/02/2020
# Last updated: 15/03/2020


def simulate(track_x, track_y, bound_xout, bound_yout, bound_xin, bound_yin, simulation_steps):
    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("mpcc_python_build_1/mpcc_optimizer")
    # Start the TCP server
    mng.start()

    # Run simulations
    phi = np.arctan2(track_x[0], track_y[0])
    x_state_0 = [track_x[0], track_y[0], phi, 0.5, 0.5, 0]
    state_sequence = x_state_0
    input_sequence = []
    state = x_state_0

    dist_ahead = (np.arctan2(x_state_0[4], x_state_0[3])) * cg.N * cg.Ts  # Using tangential velocity
    ref_rest = [1, 1, 0.7]
    i_nearest = 0
    i_ahead = move_along_track(track_x, track_y, dist_ahead, i_nearest)
    [x, y] = [track_x[i_ahead], track_y[i_ahead]]
    [x_prev, y_prev] = [track_x[i_ahead - 1], track_y[i_ahead - 1]]
    phi = np.arctan2(y - y_prev, x - x_prev)
    state_ref = np.concatenate(([x, y, phi], ref_rest))
    reference_sequence = state_ref
    bounds = [bound_xout[i_ahead], bound_yout[i_ahead], bound_xin[i_ahead], bound_yin[i_ahead]]

    for k in range(simulation_steps):
        solver_status = mng.call(np.concatenate((state, state_ref, bounds)))
        try:
            print('Loop [' + str(k) + ']: ' + str(solver_status['solve_time_ms']) + ' ms. Exit status: '
                  + solver_status['exit_status'] + '. Outer iterations: ' + str(solver_status['num_outer_iterations'])
                  + '. Inner iterations: ' + str(solver_status['num_inner_iterations']))
            us = solver_status['solution']
            u1 = us[0]
            u2 = us[1]
            forces = cg.tire_forces(state, [u1, u2])
            state_next = cg.dynamic_model_rk(np.concatenate((state, state_ref, bounds)), [u1, u2], forces, cg.Ts, False)

            # Find the index of the reference point (depending on the current velocity)
            dist_ahead = (np.arctan2(state_next[4], state_next[3])) * cg.N * cg.Ts  # Using tangential velocity
            i_nearest = find_closest_point_centreline([state_next[0], state_next[1]], track_x, track_y, cg.track_width, 30, i_nearest)
            i_ahead = move_along_track(track_x, track_y, dist_ahead, i_nearest)

            # Find the angle in the direction of the previous step - should this be from current position instead?
            [x, y] = [track_x[(i_ahead + 1 + k) % simulation_steps], track_y[(i_ahead + 1 + k) % simulation_steps]]
            [x_prev, y_prev] = [track_x[(i_ahead + k) % simulation_steps], track_y[(i_ahead + k) % simulation_steps]]
            phi = np.arctan2(y - y_prev, x - x_prev)

            # The next reference state, taking into account all the previous calculations, and using the vehicle
            # model with regard to the obtained control inputs
            state_ref = cg.dynamic_model_rk(np.concatenate((state_ref, state_ref, bounds)), [u1, u2], forces, cg.Ts, False)
            state_ref = np.concatenate(([x, y, phi], state_ref[3:6]))

            # Update all variables needed for the next iteration
            state_sequence = np.concatenate((state_sequence, state_next[:6]))
            input_sequence += [u1, u2]
            reference_sequence = np.concatenate((reference_sequence, state_ref))
            state = state_next[0:6]
            bounds = [bound_xout[i_ahead], bound_yout[i_ahead], bound_xin[i_ahead], bound_yin[i_ahead]]

        except AttributeError:
            print('Failed after ' + str(state_sequence.__len__() / cg.nx) + 'simulation steps\n'
                  + 'Error[' + str(solver_status['code']) + ']: ' + solver_status['message'])
            break
            # mng.kill()
            # sys.exit(0)

    mng.kill()

    return [input_sequence, state_sequence, reference_sequence, int((state_sequence.__len__()/cg.nx - 1))]


def find_closest_point_centreline(current_pos, track_x, track_y, track_width, search_region, prev_closest):
    # Inspired here: https://github.com/alexliniger/MPCC/blob/master/Matlab/findTheta.m
    # Find the index of the point on the centreline which is the closest to the current position

    # Investigate at search region
    back = int(search_region * 0.05)  # 5%
    front = search_region - back
    smallest_dist_i = prev_closest - (back - 1)
    smallest_dist = np.sqrt((current_pos[0] - track_x[smallest_dist_i]) ** 2 + (current_pos[1] - track_y[smallest_dist_i]) ** 2)
    for i in range(smallest_dist_i + 1, prev_closest + front):
        dist = np.sqrt((current_pos[0] - track_x[i]) ** 2 + (current_pos[1] - track_y[i]) ** 2)
        if dist < smallest_dist:
            smallest_dist = dist
            smallest_dist_i = i

    # Search through the whole track if the distance is too long
    # if smallest_dist > track_width:
    #     for i in range(len(track_x)):
    #         dist = np.sqrt((current_pos[0] - track_x[i]) ** 2 + (current_pos[1] - track_y[i]) ** 2)
    #         if dist < smallest_dist:
    #             smallest_dist = dist
    #             smallest_dist_i = i

    # Crash if still too small (for now - TODO ?)
    # if smallest_dist > track_width:
    #     sys.exit("OUT OF TRACK BOUNDARIES!")

    # TODO - wrapping around; out of boundaries error

    return smallest_dist_i


def move_along_track(track_x, track_y, distance, start_i):
    dist = distance
    i = start_i
    x = track_x[i]
    y = track_y[i]

    while dist > 0:
        i += 1
        if i >= len(track_x):
            i = 0
        x_next = track_x[i]
        y_next = track_y[i]
        dist -= np.sqrt((x_next - x) ** 2 + (y_next - y) ** 2)
        x = x_next
        y = y_next

    return i


def simulate_one_step(x_state_0, ref):
    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("mpcc_python_build_1/mpcc_optimizer")
    # Start the TCP server
    mng.start()
    # Run simulations
    state_sequence = x_state_0
    x = x_state_0

    state = np.concatenate((x, ref, [0, 0, 0, 0]))
    solver_status = mng.call(state)
    try:
        print('Result in: ' + str(solver_status['solve_time_ms']) + ' ms. Exit status: '
              + solver_status['exit_status'] + '. Outer iterations: ' + str(solver_status['num_outer_iterations'])
              + '. Inner iterations: ' + str(solver_status['num_inner_iterations']))

        input_sequence = solver_status['solution']
        for i in range(0, cg.nu * cg.N, 2):
            u1 = input_sequence[i]
            u2 = input_sequence[i + 1]
            forces = cg.tire_forces(x, [u1, u2])
            x_next = cg.dynamic_model_rk(np.concatenate((x, ref, [0, 0, 0, 0])), [u1, u2], forces, cg.Ts, False)
            state_sequence = np.concatenate((state_sequence, x_next[:6]))
            x = x_next[0:6]
    except AttributeError:
        print('Failed after ' + str(state_sequence.__len__() / cg.nx) + 'simulation steps\n'
              + 'Error[' + str(solver_status['code']) + ']: ' + solver_status['message'])
        mng.kill()
        sys.exit(0)

    mng.kill()

    return [input_sequence, state_sequence]


def plot_simulation(simulation_steps, input_sequence, state_sequence, ref):
    t = np.arange(0, cg.Ts * (simulation_steps - cg.Ts), cg.Ts)
    ref_seq = np.concatenate([ref] * t.size)

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
    plt.plot(t, ref_seq[0:cg.nx * simulation_steps:cg.nx], '--', label="x")
    plt.plot(t, ref_seq[1:cg.nx * simulation_steps:cg.nx], '--', label="y")
    plt.plot(t, ref_seq[2:cg.nx * simulation_steps:cg.nx], '--', label="phi")
    plt.grid()
    plt.ylabel('States')
    plt.xlabel('Time')
    plt.title('STATE SEQUENCE 1')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()

    plt.plot(t, state_sequence[3:cg.nx * simulation_steps:cg.nx], '-', label="v_x")
    plt.plot(t, state_sequence[4:cg.nx * simulation_steps:cg.nx], '-', label="v_y")
    plt.plot(t, state_sequence[5:cg.nx * simulation_steps:cg.nx], '-', label="omega")
    plt.plot(t, ref_seq[3:cg.nx * simulation_steps:cg.nx], '--', label="v_x")
    plt.plot(t, ref_seq[4:cg.nx * simulation_steps:cg.nx], '--', label="v_y")
    plt.plot(t, ref_seq[5:cg.nx * simulation_steps:cg.nx], '--', label="omega")
    plt.grid()
    plt.ylabel('States')
    plt.xlabel('Time')
    plt.title('STATE SEQUENCE 2')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()


def plot_track(state_ref, state_seq):
    ref_x = state_ref[0:cg.nx * state_ref.size:cg.nx]
    ref_y = state_ref[1:cg.nx * state_ref.size:cg.nx]
    state_x = state_seq[0:cg.nx * state_seq.size:cg.nx]
    state_y = state_seq[1:cg.nx * state_seq.size:cg.nx]
    fig, ax = plt.subplots(1, 1)
    ax.plot(ref_x, ref_y, 'xb', label="Reference track")
    ax.plot(state_x, state_y, 'or', label="Achieved track")
    plt.grid()
    plt.ylabel('Y position')
    plt.xlabel('X position')
    plt.title('Track')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()
