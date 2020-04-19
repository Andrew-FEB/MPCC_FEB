import sys
from warnings import warn

import matplotlib.pyplot as plt
import numpy as np
import opengen as og

import CodeGenerator as cg


# Author: Darina Abaffyová
# Created: 13/02/2020
# Last updated: 03/04/2020


def simulate(track_x, track_y, upper_bound, lower_bound, simulation_steps):
    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("mpcc_python_build_1/mpcc_optimizer")
    # Start the TCP server
    mng.start()

    # Set all values needed for simulation
    # TODO - organize these!!!
    i_start = 1  # must be at least one
    phi = np.arctan2(track_y[i_start] - track_y[i_start - 1], track_x[i_start] - track_x[i_start - 1])
    # State is a tuple which contains the six state-defining parameters
    x_state_0 = (track_x[i_start], track_y[i_start], phi, 1.5)  # , 7, 0.5)
    # x_state_0 = (1, 3, np.arctan(3/1), 0.2)  # , 7, 0.5)

    # At the end of the simulation, state sequence will contain
    # all the states that the vehicle went through during the simulation
    state_sequence = [x_state_0]
    # Input sequence will contain all the inputs calculated during the simulation, also as tuples
    input_sequence = [(np.nan, np.nan)]
    state = x_state_0

    # Here the first reference point is calculated:
    # ---------------------------------------------
    # dist_ahead is the distance that the vehicle will travel if it keeps
    # the same (tangential velocity) for the next cg.N time steps, with
    # each time step being cg.Ts second long
    # dist_ahead = (np.arctan2(x_state_0[4], x_state_0[3])) * cg.N * cg.Ts  # Using tangential velocity
    dist_ahead = x_state_0[3] * cg.N * cg.Ts
    # i_nearest is the index of the nearest point on the reference line
    i_nearest, sth = find_closest_point_centreline((x_state_0[0], x_state_0[1]), track_x, track_y, cg.track_width, 100,
                                                   0)
    [i_ahead, end_of_track_reached] = move_along_track(track_x, track_y, dist_ahead, i_nearest)
    (x, y) = [track_x[i_ahead], track_y[i_ahead]]
    (x_prev, y_prev) = [track_x[i_ahead - 1], track_y[i_ahead - 1]]
    phi = np.arctan2(y - x_state_0[1], x - x_state_0[0])
    state_ref = (x, y, phi, 1)  # , 5, 0.7)  # reference state tuple
    reference_sequence = [state_ref]

    # The nearest sequence will contain tuples (x, y) of the positions
    # on the reference line which correspond to the nearest point found
    # for each state in the state sequence
    x_nearest = track_x[i_nearest + 1]
    y_nearest = track_y[i_nearest + 1]
    slope = (y_nearest - track_y[i_nearest]) / (x_nearest - track_x[i_nearest])
    nearest_sequence = [(x_nearest, y_nearest)]

    # Get the boundaries
    x_up = x_nearest
    y_up = upper_bound[i_nearest + 1]
    slope_up = (y_up - upper_bound[i_nearest]) / (x_nearest - track_x[i_nearest])
    x_low = x_nearest
    y_low = lower_bound[i_nearest + 1]
    slope_low = (y_low - lower_bound[i_nearest]) / (x_nearest - track_x[i_nearest])

    # Run simulation
    for k in range(simulation_steps):
        solver_status = mng.call(np.concatenate((state, state_ref, [slope, x_nearest, y_nearest])))
                                                 # [slope_up, x_up, y_up, slope_low, x_low, y_low])))
        try:
            print('Loop [' + str(k) + ']: ' + str(solver_status['solve_time_ms']) + ' ms. Exit status: '
                  + solver_status['exit_status'] + '. Outer iterations: ' + str(solver_status['num_outer_iterations'])
                  + '. Inner iterations: ' + str(solver_status['num_inner_iterations'])
                  + '. Ref index = ' + str(i_ahead))

            control_inputs = solver_status['solution']
            first_control_input = (control_inputs[0], control_inputs[1])
            # forces = cg.tire_forces(state, first_control_input)
            # state_next = cg.dynamic_model_rk(np.concatenate((state, state_ref, [slope, x_nearest, y_nearest])),
            #                                  first_control_input, forces, cg.Ts, False)
            state_next = cg.kinetic_model_rk(np.concatenate((state, state_ref)),
                                             first_control_input, cg.Ts, False)
            # forces,

            end_of_track_reached, i_nearest, state_ref, i_ahead, x, y, phi = update_reference(first_control_input,
                                                                                              i_nearest, state_next,
                                                                                              state_ref, track_x,
                                                                                              track_y)
            # forces,

            if end_of_track_reached:
                break

            # Update all variables needed for the next iteration and save values in the sequences to be plotted
            x_nearest = track_x[i_nearest]
            x_nearest_prev = track_x[i_nearest - 1]
            y_nearest = track_y[i_nearest]
            y_nearest_prev = track_y[i_nearest - 1]
            slope = (y_nearest - y_nearest_prev) / (x_nearest - x_nearest_prev)

            # Update the boundaries
            x_up = x_nearest
            y_up = upper_bound[i_nearest]
            slope_up = (y_up - upper_bound[i_nearest - 1]) / (x_nearest - x_nearest_prev)
            x_low = x_nearest
            y_low = lower_bound[i_nearest]
            slope_low = (y_low - lower_bound[i_nearest - 1]) / (x_nearest - x_nearest_prev)

            # state_ref = (x, y, phi) + tuple(state_ref[3:6])
            state_ref = (x, y, phi, 2.5)  # state_ref[3]
            # state_ref = state_ref[0:6]
            # state = state_next[:6]
            state = state_next[:4]
            input_sequence.append(first_control_input)
            state_sequence.append(tuple(state_next[:4]))
            # state_sequence.append(tuple(state_next[:6]))
            reference_sequence.append(tuple(state_ref))
            nearest_sequence.append((x_nearest, y_nearest))

        except AttributeError:
            print('Failed after ' + str(len(state_sequence)) + ' simulation steps\n'
                  + 'Error[' + str(solver_status['code']) + ']: ' + solver_status['message'])
            break
            # mng.kill()
            # sys.exit(0)

    mng.kill()

    return [input_sequence, state_sequence, reference_sequence, nearest_sequence, len(state_sequence)]


def update_reference(first_control_input, i_nearest, state_next, state_ref, track_x, track_y):
    # Find the index of the reference point (depending on the current velocity), as above
    # dist_ahead = (np.arctan2(state_next[4], state_next[3])) * cg.N * cg.Ts  # Using tangential velocity
    dist_ahead = state_next[3] * 2  # = velocity * prediction horizon in seconds (cg.N * cg.Ts)
    print("DIST AHEAD = " + str(dist_ahead) + ", v = " + str(state_next[3]))
    [i_nearest, nearest_dist] = find_closest_point_centreline([state_next[0], state_next[1]], track_x, track_y,
                                                              cg.track_width,
                                                              30, i_nearest)
    [i_ahead, end_of_track_reached] = move_along_track(track_x, track_y, dist_ahead, i_nearest)
    # Find the angle in the direction of the previous step - should this be from current position instead?
    [x, y] = [track_x[i_ahead], track_y[i_ahead]]
    [x_prev, y_prev] = [track_x[i_ahead-1], track_y[i_ahead-1]]
    phi = np.arctan2(y - y_prev, x - x_prev)
    # The next reference state, taking into account all the previous calculations, and using the vehicle
    # model with regard to the obtained control inputs
    # state_ref = cg.dynamic_model_rk(np.concatenate((state_ref, state_ref, [slope, x_nearest, y_nearest])),
    #                                 first_control_input, forces, cg.Ts, False)
    state_ref = cg.kinetic_model_rk(np.concatenate((state_ref, state_ref)),
                                    first_control_input, cg.Ts, False)
    return end_of_track_reached, i_nearest, state_ref, i_ahead, x, y, phi


def find_closest_point_centreline(current_pos, track_x, track_y, track_width, search_region, prev_closest):
    # Inspired here: https://github.com/alexliniger/MPCC/blob/master/Matlab/findTheta.m
    # Find the index of the point on the centreline which is the closest to the current position

    # Investigate at search region
    back = int(search_region * 0.05)  # 5%
    # front = search_region - back
    smallest_dist_i = prev_closest - (back - 1)
    if smallest_dist_i < 0:
        smallest_dist_i = len(track_x) + smallest_dist_i
    smallest_dist = np.sqrt(
        (current_pos[0] - track_x[smallest_dist_i]) ** 2 + (current_pos[1] - track_y[smallest_dist_i]) ** 2)
    for i in range(smallest_dist_i + 1, smallest_dist_i + search_region):
        ii = i % len(track_x)
        dist = np.sqrt((current_pos[0] - track_x[ii]) ** 2 + (current_pos[1] - track_y[ii]) ** 2)
        if dist < smallest_dist:
            smallest_dist = dist
            smallest_dist_i = ii

    # Search through the whole track if the distance is too long
    if smallest_dist > track_width:
        for i in range(len(track_x)):
            dist = np.sqrt((current_pos[0] - track_x[i]) ** 2 + (current_pos[1] - track_y[i]) ** 2)
            if dist < smallest_dist:
                smallest_dist = dist
                smallest_dist_i = i

    # Crash if still too big (for now - TODO ?)
    if smallest_dist > track_width:
        warn("OUT OF TRACK BOUNDARIES!")
        # sys.exit("OUT OF TRACK BOUNDARIES!")

    # TODO - wrapping around; out of boundaries error

    return [smallest_dist_i, smallest_dist]


# This function calculates the next reference point which
# is distance far from the point (track_x[start_i], track_y[start_i]).
def move_along_track(track_x, track_y, distance, start_i):
    dist = distance
    i = start_i
    x = track_x[i]
    y = track_y[i]

    end_of_track_reached = False

    while dist > 0:
        i += 1
        if i >= len(track_x):
            i = len(track_x) - 1
            end_of_track_reached = True
            warn("INDEX RESTART!!!")
            break
        x_next = track_x[i]
        y_next = track_y[i]
        dist -= np.sqrt((x_next - x) ** 2 + (y_next - y) ** 2)
        x = x_next
        y = y_next

    return [i, end_of_track_reached]


def simulate_one_step(x_state_0, ref):
    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("mpcc_python_build_1/mpcc_optimizer")
    # Start the TCP server
    mng.start()
    # Run simulations
    state_sequence = [tuple(x_state_0)]
    state = x_state_0

    slope = ref[1] / ref[0]
    input_sequence = [(np.nan, np.nan)]

    state_next = state
    state = np.concatenate((state, ref, [slope, ref[0], ref[1]]))
    solver_status = mng.call(state)

    states = []
    try:
        print('Result in: ' + str(solver_status['solve_time_ms']) + ' ms. Exit status: '
              + solver_status['exit_status'] + '. Outer iterations: ' + str(solver_status['num_outer_iterations'])
              + '. Inner iterations: ' + str(solver_status['num_inner_iterations']))

        controls_sequence = solver_status['solution']
        for i in range(0, cg.nu * cg.N, 2):
            D = controls_sequence[i]
            delta = controls_sequence[i + 1]
            forces = cg.tire_forces(state, [D, delta])
            state_next = cg.dynamic_model_rk(np.concatenate((state_next[0:6], ref, [slope, ref[0], ref[1]])),
                                             [D, delta],
                                             forces, cg.Ts, False)

            states.append(state_next)

            state_sequence.append(tuple(state_next[:6]))
            input_sequence.append(tuple([D, delta]))
    except AttributeError:
        print('Failed after ' + str(len(state_sequence)) + ' simulation steps\n'
              + 'Error[' + str(solver_status['code']) + ']: ' + solver_status['message'])
        mng.kill()
        sys.exit(0)

    mng.kill()

    plot_cost(states, [100, 10])

    return [input_sequence, state_sequence]


def plot_simulation(simulation_steps, input_seq, state_seq, ref_seq):
    t = np.arange(0, cg.Ts * (simulation_steps - cg.Ts), cg.Ts)  # - cg.Ts

    plt.plot(t, [D for D, delta in input_seq], '-', label="Acceleration")
    plt.plot(t, [delta for D, delta in input_seq], '-', label="Front steering angle")
    plt.grid()
    plt.ylabel('Input')
    plt.xlabel('Time')
    plt.title('INPUT SEQUENCE')
    plt.legend(bbox_to_anchor=(0.7, 0.85), loc='best', borderaxespad=0.)
    plt.show()

    plt.plot(t, [x for x, *_ in state_seq], '-', label="x")
    plt.plot(t, [y for x, y, *_ in state_seq], '-', label="y")
    # plt.plot(t, [phi for x, y, phi, *_ in state_seq], '-', label="phi")
    plt.plot(t, [x for x, *_ in ref_seq], '--', label="x")
    plt.plot(t, [y for x, y, *_ in ref_seq], '--', label="y")
    # plt.plot(t, [phi for x, y, phi, *_ in ref_seq], '--', label="phi")
    plt.grid()
    plt.ylabel('States')
    plt.xlabel('Time')
    plt.title('STATE SEQUENCE 1')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()

    plt.plot(t, [phi for x, y, phi, v in state_seq], '-', label="phi")
    plt.plot(t, [v for x, y, phi, v in state_seq], '-', label="v")
    plt.plot(t, [phi for x, y, phi, v in ref_seq], '--', label="phi")
    plt.plot(t, [v for x, y, phi, v in ref_seq], '--', label="v")

    # plt.plot(t, [v_x for x, y, phi, v_x, *_ in state_seq], '-', label="v_x")
    # plt.plot(t, [v_y for x, y, phi, v_x, v_y, omega in state_seq], '-', label="v_y")
    # plt.plot(t, [omega for x, y, phi, v_x, vy, omega in state_seq], '-', label="omega")
    # plt.plot(t, [v_x for x, y, phi, v_x, *_ in ref_seq], '--', label="v_x")
    # plt.plot(t, [v_y for x, y, phi, v_x, v_y, omega in ref_seq], '--', label="v_y")
    # plt.plot(t, [omega for x, y, phi, v_x, vy, omega in ref_seq], '--', label="omega")
    plt.grid()
    plt.ylabel('States')
    plt.xlabel('Time')
    plt.title('STATE SEQUENCE 2')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()


def plot_track(track_x, track_y, upper, lower, ref_seq, state_seq):
    ref_x = [x for x, *_ in ref_seq]
    ref_y = [y for x, y, *_ in ref_seq]
    state_x = [x for x, *_ in state_seq]
    state_y = [y for x, y, *_ in state_seq]
    fig, ax = plt.subplots(1, 1)
    ax.plot(track_x, track_y, '.y', label="Complete track")
    ax.plot(track_x, upper, '--g', label="Boundaries")
    ax.plot(track_x, lower, '--g')
    ax.plot(state_x, state_y, 'or', label="Achieved track")
    ax.plot(ref_x, ref_y, 'xb', label="Reference track")
    plt.grid()
    plt.ylabel('Y position')
    plt.xlabel('X position')
    plt.title('Track')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()


def plot_track2(track_x, track_y, ref_seq, state_seq):
    ref_x = [x for x, *_ in ref_seq]
    ref_y = [y for x, y, *_ in ref_seq]
    state_x = [x for x, *_ in state_seq]
    state_y = [y for x, y, *_ in state_seq]
    fig, ax = plt.subplots(1, 1)
    ax.plot(track_x, track_y, '.y', label="Complete track")
    ax.plot(state_x, state_y, 'or', label="Achieved track")
    ax.plot(ref_x, ref_y, 'xb', label="Reference track")
    plt.grid()
    plt.ylabel('Y position')
    plt.xlabel('X position')
    plt.title('Track')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()


def plot_nearest(track_x, track_y, nearest_seq, state_seq):
    state_x = [x for x, *_ in state_seq]
    state_y = [y for x, y, *_ in state_seq]
    nearest_x = [x for x, y, *_ in nearest_seq]
    nearest_y = [y for x, y, *_ in nearest_seq]

    fig, ax = plt.subplots(1, 1)
    ax.plot(track_x, track_y, '.y', label="Complete track")

    for i in range(0, len(state_x)):
        ax.plot([nearest_x[i], state_x[i]], [nearest_y[i], state_y[i]], '-k')

    ax.plot(state_x, state_y, 'or', label="Achieved track")
    ax.plot(nearest_x, nearest_y, 'xb', label="Nearest points")

    i = 0
    for x, y in zip(state_x, state_y):
        dist = round(np.sqrt((x - nearest_x[i]) ** 2 + (y - nearest_y[i]) ** 2), 3)
        plt.text(x, y, str(dist), color="red", fontsize=10)
        i += 1

    plt.axis('equal')
    plt.grid()
    plt.ylabel('Y position')
    plt.xlabel('X position')
    plt.title('Track')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()


def plot_cost(states, contouring_error_weight):
    c = [0] * len(states)
    for i in range(len(states)):
        c[i] = cg.cost_function(states[i], contouring_error_weight)

    t = np.arange(0, cg.Ts * (len(c) - cg.Ts), cg.Ts)

    plt.plot(t, c)
    plt.grid()
    plt.ylabel('Cost')
    plt.xlabel('Time')
    plt.title('Cost progress')
    plt.show()
