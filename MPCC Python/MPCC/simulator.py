import sys
from collections import namedtuple
from warnings import warn
import random

import numpy as np
import opengen as og

import codegenerator as cg
import parameters as param

# Author: Darina Abaffyová
# Created: 13/02/2020

# Define all named tuples here
KinematicState = namedtuple('KinematicState', 'x y psi v')
DynamicState = namedtuple('DynamicState', 'x y phi v_x v_y omega')
Control = namedtuple('Control', 'acc delta')
Intercept = namedtuple('intercept', 'left right')
Slope = namedtuple('slope', 'left right')


def simulate(track_x, track_y, left_x, left_y, right_x, right_y, simulation_steps):
    # Set all values needed for simulation
    i_start = 0
    # State is a tuple which contains the four state-defining parameters
    x = track_x[i_start]
    y = track_y[i_start]
    x_next = track_x[i_start + 1]
    y_next = track_y[i_start + 1]
    v = 0
    psi = np.arctan2(y_next - y, x_next - x)
    state_0 = KinematicState(x, y, psi, v)
    state = state_0

    # Here the reference list is calculated:
    # ---------------------------------------------
    # i_nearest is the index of the nearest point on the reference line
    i_nearest = get_nearest_point((state_0.x, state_0.y), track_x, track_y, int(len(track_x) * 0.1), 0)
    i_nearest_prev = i_nearest
    ref_list, ref_indexes, end_reached = get_reference_list(track_x, track_y, i_nearest)
    intercepts, slopes, track_width, i_left, i_right = get_boundaries(left_y, left_x, right_y, right_x, ref_indexes,
                                                                      ref_list)
    # On the first iteration, 0s are used as the initial guess, afterwards the previous output is used
    control_inputs = [0] * param.N * param.nu

    # At the end of the simulation, a number of sequences is outputted and used for plotting
    # state_seq will contain all the states that the vehicle went through during the simulation
    state_seq = [state_0]
    # control_inputs_seq will contain all the lists of control inputs outputted by the solver
    control_inputs_seq = []
    # first_control_seq will contain all the inputs given to the car during the simulation as tuples
    first_control_seq = [(np.nan, np.nan)]
    # ref_seq will contain all lists of references used
    ref_seq = [ref_list]
    # bound_seq will contain all the boundaries given to the solver
    bound_seq = [(slopes, intercepts, track_width)]
    # cost_seq, solve_time_seq and exit_status_seq will contain some solver metadata
    cost_seq = [0]
    solve_time_seq = [0]
    exit_status_seq = ['']

    not_conv_time = 0
    not_conv_iter = 0

    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("mpcc_python_build_kin/mpcc_optimizer")
    # Start the TCP server
    mng.start()

    # Run simulation
    for k in range(simulation_steps):
        solver_status = mng.call(np.concatenate((state, [slopes.left, slopes.right, intercepts.left, intercepts.right],
                                                 np.reshape(ref_list, 2 * param.N))), control_inputs)
        print("Parameters: " + str(
            np.concatenate((state, [slopes.left, slopes.right, intercepts.left, intercepts.right,
                                    track_width]))))
        try:
            print('Loop [' + str(k) + ']: ' + str(solver_status['solve_time_ms']) + ' ms. Exit status: '
                  + solver_status['exit_status'] + '. Outer iterations: ' + str(solver_status['num_outer_iterations'])
                  + '. Inner iterations: ' + str(solver_status['num_inner_iterations'])
                  + '. Penalty: ' + str(solver_status['penalty'])
                  + '. Nearest index = ' + str(i_nearest)
                  + '. Track width = ' + str(track_width))
            # Count the not converged ones
            if solver_status['exit_status'] == 'NotConvergedOutOfTime':
                not_conv_time += 1
            elif solver_status['exit_status'] == 'NotConvergedIterations':
                not_conv_iter += 1

            # Get the solution
            control_inputs = solver_status['solution']
            # Only the first controls will be given to the car
            first_control_input = Control(control_inputs[0], control_inputs[1])
            # Update the state
            state_next = cg.kinematic_model_rk(state, first_control_input, False)
            i_nearest = get_nearest_point(state_next[0:2], track_x, track_y, int(len(track_x) * 0.1), 0)
            ref_list, ref_indexes, end_of_track_reached = get_reference_list(track_x, track_y, i_nearest)
            intercepts, slopes, track_width, i_left, i_right = get_boundaries(left_y, left_x, right_y, right_x,
                                                                              ref_indexes, ref_list)

            # Enable this for non-circular tracks
            # if end_of_track_reached and k > 5: break
            if i_nearest_prev > i_nearest and k > 5:
                warn("END OF TRACK REACHED!")
                break
            i_nearest_prev = i_nearest

            # Update all variables needed for the next iteration and save values in the sequences to be plotted
            # Introduce noise to the state parameters
            # state = KinematicState(state_next[0] + random.uniform(-0.05, 0.05),  # x
            #                        state_next[1] + random.uniform(-0.05, 0.05),  # y
            #                        state_next[2] + random.uniform(-0.001, 0.001),  # psi
            #                        state_next[3] + random.uniform(-0.01, 0.01))  # v
            state = KinematicState(state_next[0],  # x
                                   state_next[1],  # y
                                   state_next[2],  # psi
                                   state_next[3])  # v
            first_control_seq.append(first_control_input)
            state_seq.append(state)
            ref_seq.append(ref_list)
            bound_seq.append((slopes, intercepts, track_width))
            cost_seq.append(solver_status['cost'])
            solve_time_seq.append(solver_status['solve_time_ms'])
            exit_status_seq.append(solver_status['exit_status'])
            control_inputs_seq.append(control_inputs)

            print("STATE = " + str(state))
            print("CONTROL = " + str(first_control_input))

        except AttributeError:
            print('Failed after ' + str(len(state_seq)) + ' simulation steps\n'
                  + 'Error[' + str(solver_status['code']) + ']: ' + solver_status['message'])
            break

    mng.kill()

    print("------------- SIMULATION SUMMARY -------------")
    track_widths = [tw for s, i, tw in bound_seq]
    print("[Min, Max, Avg] track width = [" + str(min(track_widths)) + ", " + str(max(track_widths)) + ", "
          + str(np.mean(track_widths)) + "]")
    slopes = [s.left for s, *_ in bound_seq]
    print("[Min, Max, Avg] slope left = [" + str(min(slopes)) + ", " + str(max(slopes)) + ", "
          + str(np.mean(slopes)) + "]")
    slopes_right = [s.right for s, *_ in bound_seq]
    print("[Min, Max, Avg] slope right = [" + str(min(slopes_right)) + ", " + str(max(slopes_right)) + ", "
          + str(np.mean(slopes_right)) + "]")
    print("NotConvergedOutOfTime: " + str(not_conv_time)
          + " times\nNotConvergedIterations: " + str(not_conv_iter) + " times")

    # Calculate the travelled distance
    x_prev = state_seq[0].x
    y_prev = state_seq[0].y
    dist_travelled = 0
    for i in range(len(state_seq)):
        xt = state_seq[i].x
        yt = state_seq[i].y
        dist_travelled += round(np.sqrt((xt - x_prev) ** 2 + (yt - y_prev) ** 2), 3)
        x_prev = xt
        y_prev = yt
    print("Complete travelled distance: " + str(dist_travelled))

    return state_seq, ref_seq, bound_seq, cost_seq, solve_time_seq, exit_status_seq, first_control_seq, \
           control_inputs_seq, len(state_seq)


def get_boundaries(left_y, left_x, right_y, right_x, indexes, ref_list):
    # Take the boundary tangents at i = N/2
    i = int(param.N / 2)
    i_left = get_nearest_point(ref_list[i], left_x, left_y, int(len(right_x) * 0.1), indexes[i])
    if i_left > len(left_x) - 2: i_left = len(left_x) - 2
    i_right = get_nearest_point(ref_list[i], right_x, right_y, int(len(right_x) * 0.1), indexes[i])
    if i_right > len(right_x) - 2: i_right = len(right_x) - 2

    # Upper boundary
    x_left = left_x[i_left]
    x_left_next = left_x[i_left + 1]
    y_left = left_y[i_left]
    y_left_next = left_y[i_left + 1]
    slope_left = (y_left_next - y_left) / (x_left_next - x_left)
    intercept_left = y_left - slope_left * x_left

    # Lower boundary
    x_right = right_x[i_right]
    x_right_next = right_x[i_right + 1]
    y_right = right_y[i_right]
    y_right_next = right_y[i_right + 1]
    slope_right = (y_right_next - y_right) / (x_right_next - x_right)
    intercept_right = y_right - slope_right * x_right

    intercepts = Intercept(intercept_left, intercept_right)
    slopes = Slope(slope_left, slope_right)
    track_width = np.sqrt((x_left - x_right) ** 2 + (y_left - y_right) ** 2)

    return intercepts, slopes, track_width, i_left, i_right


# Construct a reference trajectory (within the prediction horizon!) instead of a single point (either using
# the maximum velocity or the current velocity). So starting from the current position, the reference position
# for timestep 1 will be the closest point on the path to the current position + Ts * current velocity,
# after timestep 2, it will be that point + Ts * (current velocity (+ some acceleration) ), where "some
# acceleration" could be either the maximum acceleration or 0. Here, the maximum is used.
def get_reference_list(track_x, track_y, i_nearest):
    x = [track_x[i_nearest]]
    y = [track_y[i_nearest]]
    dist = param.Ts * param.v_x_max
    j = i_nearest
    dist_prev = dist
    indexes = [j]
    end_reached = False

    for i in range(0, param.N):
        d = dist_prev + param.a_max * param.Ts ** 2
        dist_prev = d
        x_temp = x[len(x) - 1]
        y_temp = y[len(y) - 1]
        while d > 0:
            j += 1
            x_next = track_x[min(j, len(track_x) - 1)]
            y_next = track_y[min(j, len(track_y) - 1)]
            d -= np.sqrt((x_next - x_temp) ** 2 + (y_next - y_temp) ** 2)
            x_temp = x_next
            y_temp = y_next
            if j >= len(track_x) - 3:
                j = 0
        if i >= 1:
            x = np.concatenate([x, [x_temp]])
            y = np.concatenate([y, [y_temp]])
            indexes = np.concatenate([indexes, [j]])

    return list(zip(x, y)), indexes, end_reached


# Inspired here: https://github.com/alexliniger/MPCC/blob/master/Matlab/findTheta.m
# Find the index of the point on the centreline which is the closest to the current position
def get_nearest_point(current_pos, track_x, track_y, search_region, prev_closest):
    track_width = param.track_width / 2

    # Investigate at search region
    back = int(search_region * 0.05)  # 5%
    # front = search_region - back
    smallest_dist_i = prev_closest - (back - 1)
    if smallest_dist_i < 0:
        smallest_dist_i = len(track_x) + smallest_dist_i
    smallest_dist = np.sqrt((current_pos[0] - track_x[smallest_dist_i]) ** 2
                            + (current_pos[1] - track_y[smallest_dist_i]) ** 2)
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

    if smallest_dist > track_width:
        warn("OUT OF TRACK BOUNDARIES!")
        # sys.exit("OUT OF TRACK BOUNDARIES!")

    return smallest_dist_i
