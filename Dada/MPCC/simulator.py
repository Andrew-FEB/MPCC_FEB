import sys
from collections import namedtuple
from warnings import warn

import numpy as np
import opengen as og

import codegenerator as cg
import parameters as param

# Author: Darina Abaffyová
# Created: 13/02/2020
# Last updated: 03/07/2020

# Define all named tuples here
KinematicState = namedtuple('KinematicState', 'x y psi v')
DynamicState = namedtuple('DynamicState', 'x y phi v_x v_y omega')
Control = namedtuple('Control', 'D delta')
Intercept = namedtuple('intercept', 'upper lower')
Slope = namedtuple('slope', 'upper lower')


def simulate(track_x, track_y, upper, lower, simulation_steps):
    # Set all values needed for simulation
    # TODO - organize these!!!
    i_start = 0
    # State is a tuple which contains the four state-defining parameters
    x = track_x[i_start]
    y = track_y[i_start]
    v_x = 1
    v_y = 1
    psi = np.arctan2(v_y, v_x)  # TODO - is this rate or just the angle???
    state_0 = KinematicState(x, y, psi, v_x)

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

    # The nearest sequence will contain tuples (x, y) of the positions
    # on the reference line which correspond to the nearest point found
    # for each state in the state sequence
    intercepts, slopes, track_widths = get_boundaries(lower, track_x, upper, ref_indexes, ref_list)
    bound_seq = [(slopes, intercepts, ref_indexes)]

    control_inputs = [0] * param.N * param.nu  # warm_start(state, ref_state, [slope, x_nearest, y_nearest])
    control_inputs_seq = []

    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("mpcc_python_build_2/mpcc_optimizer")
    # Start the TCP server
    mng.start()

    # Run simulation
    for k in range(simulation_steps):
        solver_status = mng.call(
            np.concatenate((state, np.reshape(slopes, 2 * param.N), np.reshape(intercepts, 2 * param.N)
                            , track_widths, np.reshape(ref_list, 2 * param.N))), control_inputs)

        try:
            print('Loop [' + str(k) + ']: ' + str(solver_status['solve_time_ms']) + ' ms. Exit status: '
                  + solver_status['exit_status'] + '. Outer iterations: ' + str(solver_status['num_outer_iterations'])
                  + '. Inner iterations: ' + str(solver_status['num_inner_iterations'])
                  + '. Penalty: ' + str(solver_status['penalty'])
                  + '. Nearest index = ' + str(i_nearest))

            control_inputs = solver_status['solution']
            first_control_input = Control(control_inputs[0], control_inputs[1])
            state_next = cg.kinematic_model_rk(state, first_control_input, False)
            i_nearest = get_nearest_point(state_next[0:2], track_x, track_y, int(len(track_x) * 0.1), 0)
            ref_list, ref_indexes, end_of_track_reached = get_reference_list(track_x, track_y, i_nearest)

            if end_of_track_reached: break

            # Update all variables needed for the next iteration and save values in the sequences to be plotted
            intercepts, slopes, track_widths = get_boundaries(lower, track_x, upper, ref_indexes, ref_list)

            # ref_state = get_reference(i_nearest, i_ahead, track_x, track_y)
            state = KinematicState(state_next[0], state_next[1], state_next[2], state_next[3])
            first_control_seq.append(first_control_input)
            state_seq.append(state)
            ref_seq.append(ref_list)
            bound_seq.append((slopes, intercepts, ref_indexes))
            cost_seq.append(solver_status['cost'])
            control_inputs_seq.append(control_inputs)

            print("STATE = " + str(state))
            print("CONTROL = " + str(first_control_input))

        except AttributeError:
            print('Failed after ' + str(len(state_seq)) + ' simulation steps\n'
                  + 'Error[' + str(solver_status['code']) + ']: ' + solver_status['message'])
            break

    mng.kill()
    return state_seq, ref_seq, bound_seq, cost_seq, first_control_seq, control_inputs_seq, len(state_seq)


def get_boundaries(lower, track_x, upper, indexes, ref_list):
    # A set of slopes and intercepts will be needed that will span the whole prediction horizon
    intercepts = []
    slopes = []
    track_widths = []

    for i in range(len(ref_list)):
        i_up = get_nearest_point(ref_list[i], track_x, upper, int(len(track_x) * 0.1), indexes[i])
        if i_up > len(track_x) - 2: i_up = len(track_x) - 2
        i_low = get_nearest_point(ref_list[i], track_x, lower, int(len(track_x) * 0.1), indexes[i])
        if i_low > len(track_x) - 2: i_low = len(track_x) - 2

        # Upper boundary
        x_up = track_x[i_up]
        x_up_next = track_x[i_up + 1]
        y_up = upper[i_up]
        y_up_next = upper[i_up + 1]
        slope_up = (y_up_next - y_up) / (x_up_next - x_up)
        intercept_up = y_up - slope_up * x_up

        # Lower boundary
        x_low = track_x[i_low]
        x_low_next = track_x[i_low + 1]
        y_low = lower[i_low]
        y_low_next = lower[i_low + 1]
        slope_low = (y_low_next - y_low) / (x_low_next - x_low)
        intercept_low = y_low - slope_low * x_low

        intercepts.append(Intercept(intercept_up, intercept_low))
        slopes.append(Slope(slope_up, slope_low))
        track_widths.append(np.sqrt((x_up - x_low) ** 2 + (y_up - y_low) ** 2))

    return intercepts, slopes, track_widths


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

    for i in range(0, param.N - 1):
        d = dist_prev + 2 * param.a_max * param.Ts ** 2
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
                end_reached = True
                break
        x = np.concatenate([x, [x_temp]])
        y = np.concatenate([y, [y_temp]])
        indexes = np.concatenate([indexes, [j]])
        if j >= len(track_x) - 3:
            end_reached = True
            break

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
    track_width = param.track_width

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

    # Crash if still too big (for now - TODO ?)
    if smallest_dist > track_width / 2:
        warn("OUT OF TRACK BOUNDARIES!")
        # sys.exit("OUT OF TRACK BOUNDARIES!")

    # TODO - wrapping around; out of boundaries error

    return smallest_dist_i


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
