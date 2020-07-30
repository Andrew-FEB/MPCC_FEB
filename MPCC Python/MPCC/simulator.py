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
    # TODO - organize these!!!
    i_start = 2000
    # State is a tuple which contains the four state-defining parameters
    x = track_x[i_start]
    y = track_y[i_start]
    x_next = track_x[i_start + 1]
    y_next = track_y[i_start + 1]
    v = 0
    psi = np.arctan2(y_next - y, x_next - x)
    state_0 = KinematicState(x, y, psi, v)

    # At the end of the simulation, state sequence will contain
    # all the states that the vehicle went through during the simulation
    state_seq = [state_0]
    # Input sequence will contain all the inputs calculated during the simulation, also as tuples
    first_control_seq = [(np.nan, np.nan)]
    state = state_0

    # Here the first reference point is calculated:
    # ---------------------------------------------
    # dist_ahead is the distance that the vehicle will travel if it keeps
    # the same (tangential) velocity for the next param.N time steps, with
    # each time step being param.Ts seconds long
    # i_nearest is the index of the nearest point on the reference line
    i_nearest = get_nearest_point((state_0.x, state_0.y), track_x, track_y, int(len(track_x) * 0.1), 0)
    ref_list, ref_indexes, end_reached = get_reference_list(track_x, track_y, i_nearest)
    ref_seq = [ref_list]

    cost_seq = [0]
    solve_time_seq = [0]
    exit_status_seq = ['']

    # The nearest sequence will contain tuples (x, y) of the positions
    # on the reference line which correspond to the nearest point found
    # for each state in the state sequence
    intercepts, slopes, track_width, i_left, i_right = get_boundaries(left_y, left_x, right_y, right_x, ref_indexes,
                                                                      ref_list)
    bound_seq = [(slopes, intercepts, track_width)]

    control_inputs = [0] * param.N * param.nu  # warm_start(state, ref_state, [slope, x_nearest, y_nearest])
    control_inputs_seq = []

    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("mpcc_python_build_kin/mpcc_optimizer")
    # Start the TCP server
    mng.start()

    # Run simulation
    for k in range(simulation_steps):
        solver_status = mng.call(np.concatenate((state, [slopes.left, slopes.right, intercepts.left, intercepts.right,
                                                         2*track_width],
                                                 np.reshape(ref_list, 2 * param.N))), control_inputs)

        try:
            print('Loop [' + str(k) + ']: ' + str(solver_status['solve_time_ms']) + ' ms. Exit status: '
                  + solver_status['exit_status'] + '. Outer iterations: ' + str(solver_status['num_outer_iterations'])
                  + '. Inner iterations: ' + str(solver_status['num_inner_iterations'])
                  + '. Penalty: ' + str(solver_status['penalty'])
                  + '. Nearest index = ' + str(i_nearest)
                  + '. Track width = ' + str(track_width))

            control_inputs = solver_status['solution']
            first_control_input = Control(control_inputs[0], control_inputs[1])
            state_next = cg.kinematic_model_rk(state, first_control_input, False)
            i_nearest = get_nearest_point(state_next[0:2], track_x, track_y, int(len(track_x) * 0.1), 0)
            ref_list, ref_indexes, end_of_track_reached = get_reference_list(track_x, track_y, i_nearest)

            # TODO - enable this for non-circular tracks
            # if end_of_track_reached: break

            # Update all variables needed for the next iteration and save values in the sequences to be plotted
            intercepts, slopes, track_width, i_left, i_right = get_boundaries(left_y, left_x, right_y, right_x,
                                                                              ref_indexes, ref_list)

            # Noise added to the state parameters
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
    return state_seq, ref_seq, bound_seq, cost_seq, solve_time_seq, exit_status_seq, first_control_seq,\
           control_inputs_seq, len(state_seq)


def get_boundaries(left_y, left_x, right_y, right_x, indexes, ref_list):
    # A set of slopes and intercepts will be needed that will span the whole prediction horizon
    # intercepts = []
    # slopes = []
    # track_width = []

    # for i in range(len(ref_list)):
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
# acceleration" could be either the maximum acceleration or 0. You can quite efficiently compute such a sequence
# of points using the cumsum function in numpy.
def get_reference_list(track_x, track_y, i_nearest):
    x = [track_x[i_nearest]]
    y = [track_y[i_nearest]]
    dist = param.Ts * param.v_x_max
    j = i_nearest
    dist_prev = dist
    indexes = [j]
    end_reached = False

    for i in range(0, param.N):
        d = dist_prev + 0.5 * param.a_max * param.Ts ** 2
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
                end_reached = True
                # break
        if i >= 1:
            x = np.concatenate([x, [x_temp]])
            y = np.concatenate([y, [y_temp]])
            indexes = np.concatenate([indexes, [j]])
        if j >= len(track_x) - 3:
            j = 0
            end_reached = True
            # break

    if end_reached: warn("END OF TRACK REACHED!")

    return list(zip(x, y)), indexes, end_reached


def warm_start(state, state_ref, bound):
    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("mpcc_python_build_2/mpcc_optimizer")
    # Start the TCP server
    mng.start()

    s = [state[0], state[1], state[5], state[3]]
    sr = [state_ref[0], state_ref[1], state_ref[5], state_ref[3]]

    resp = mng.call(np.concatenate((s, sr, bound)))

    print('Warm start: ' + str(resp['solve_time_ms']) + ' ms. Exit status: '
          + resp['exit_status'] + '. Outer iterations: ' + str(resp['num_outer_iterations'])
          + '. Inner iterations: ' + str(resp['num_inner_iterations'])
          + '. Penalty: ' + str(resp['penalty']))

    mng.kill()

    return resp['solution']


def get_nearest_point(current_pos, track_x, track_y, search_region, prev_closest):
    # Inspired here: https://github.com/alexliniger/MPCC/blob/master/Matlab/findTheta.m
    # Find the index of the point on the centreline which is the closest to the current position
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
    # if smallest_dist > track_width:
    for i in range(len(track_x)):
        dist = np.sqrt((current_pos[0] - track_x[i]) ** 2 + (current_pos[1] - track_y[i]) ** 2)
        if dist < smallest_dist:
            smallest_dist = dist
            smallest_dist_i = i

    if smallest_dist > track_width:
        warn("OUT OF TRACK BOUNDARIES!")
        # sys.exit("OUT OF TRACK BOUNDARIES!")

    return smallest_dist_i
