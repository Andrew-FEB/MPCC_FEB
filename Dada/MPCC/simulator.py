from warnings import warn
import numpy as np
import opengen as og
import codegenerator as cg
import parameters as param


# Author: Darina Abaffyová
# Created: 13/02/2020
# Last updated: 03/04/2020

def simulate(track_x, track_y, simulation_steps):
    # Set all values needed for simulation
    # TODO - organize these!!!
    i_start = 0
    # State is a tuple which contains the four state-defining parameters
    x = track_x[i_start]
    y = track_y[i_start]
    phi = np.arctan2(y, x)
    v_x = 3
    v_y = 1
    omega = np.arctan2(v_y, v_x)
    x_state_0 = (x, y, omega, v_x)  # , v_y, omega)

    # At the end of the simulation, state sequence will contain
    # all the states that the vehicle went through during the simulation
    state_seq = [x_state_0]
    # Input sequence will contain all the inputs calculated during the simulation, also as tuples
    input_seq = [(np.nan, np.nan)]
    state = x_state_0

    # Here the first reference point is calculated:
    # ---------------------------------------------
    # dist_ahead is the distance that the vehicle will travel if it keeps
    # the same (tangential) velocity for the next cg.N time steps, with
    # each time step being cg.Ts second long
    dist_ahead = x_state_0[3] * param.N * param.Ts
    # i_nearest is the index of the nearest point on the reference line
    i_nearest, dummy = get_nearest_point((x_state_0[0], x_state_0[1]), track_x, track_y, param.track_width, 300, 0)
    i_ahead, dummy = move_along_track(track_x, track_y, dist_ahead, i_nearest)
    (x, y) = [track_x[i_ahead], track_y[i_ahead]]
    phi = np.arctan2(y, x)
    v_x = 1
    v_y = 0
    omega = 0  # np.arctan2(v_y, v_x)
    state_ref = (x, y, omega, v_x)  # , v_y, omega)  # reference state tuple
    reference_seq = [state_ref]

    cost_seq = [0]

    # The nearest sequence will contain tuples (x, y) of the positions
    # on the reference line which correspond to the nearest point found
    # for each state in the state sequence
    i_nearest = i_nearest + round((i_ahead-i_nearest)/4)
    x_nearest = track_x[i_nearest]
    y_nearest = track_y[i_nearest]
    slope = (track_y[i_nearest + 1] - y_nearest) / (track_x[i_nearest + 1] - x_nearest)
    # y = slope * x + y_intercept
    intercept = y_nearest - slope * x_nearest
    nearest_seq = [(slope, intercept, i_nearest)]

    control_inputs = [0] * param.N * param.nu  # warm_start(state, state_ref, [slope, x_nearest, y_nearest])
    control_inputs_seq = []

    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("mpcc_python_build_2/mpcc_optimizer")
    # Start the TCP server
    mng.start()

    # Run simulation
    for k in range(simulation_steps):
        solver_status = mng.call(np.concatenate((state, state_ref, [slope, intercept])), control_inputs)

        try:
            print('Loop [' + str(k) + ']: ' + str(solver_status['solve_time_ms']) + ' ms. Exit status: '
                  + solver_status['exit_status'] + '. Outer iterations: ' + str(solver_status['num_outer_iterations'])
                  + '. Inner iterations: ' + str(solver_status['num_inner_iterations'])
                  + '. Penalty: ' + str(solver_status['penalty'])
                  + '. Ref index = ' + str(i_ahead) + '. Nearest index = ' + str(i_nearest))

            control_inputs = solver_status['solution']
            first_control_input = (control_inputs[0], control_inputs[1])
            state_next = cg.kinetic_model_rk(state, first_control_input, False)

            end_of_track_reached, i_nearest, state_ref, i_ahead, x, y = update_reference(first_control_input,
                                                                                         i_nearest, state_next,
                                                                                         state_ref, track_x,
                                                                                         track_y)
            if end_of_track_reached: break

            # Update all variables needed for the next iteration and save values in the sequences to be plotted
            # i_nearest = i_nearest + round((i_ahead - i_nearest))
            x_nearest = track_x[i_nearest]
            x_nearest_next = track_x[i_nearest + 1]  # len(track_x) - 1 if i_nearest + 1 >= len(track_x) else
            y_nearest = track_y[i_nearest]
            y_nearest_next = track_y[i_nearest + 1]  # len(track_y) - 1 if i_nearest + 1 >= len(track_y) else
            slope = (y_nearest_next - y_nearest) / (x_nearest_next - x_nearest)
            # y = slope * x + y_intercept
            intercept = y_nearest - slope * x_nearest

            state_ref = (x, y, state_ref[2], state_ref[3])
            state = tuple(state_next[:4])
            print("STATE = " + str(state))
            input_seq.append(first_control_input)
            print("CONTROL = " + str(first_control_input))
            state_seq.append(state)
            reference_seq.append(state_ref)
            nearest_seq.append((slope, intercept, i_nearest))
            cost_seq.append(solver_status['cost'])
            control_inputs_seq.append(control_inputs)

        except AttributeError:
            print('Failed after ' + str(len(state_seq)) + ' simulation steps\n'
                  + 'Error[' + str(solver_status['code']) + ']: ' + solver_status['message'])
            break

    mng.kill()
    return state_seq, reference_seq, nearest_seq, cost_seq, input_seq, control_inputs_seq, len(state_seq)


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


def update_reference(first_control_input, i_nearest, state_next, state_ref, track_x, track_y):
    # Find the index of the reference point (depending on the current velocity), as above
    v = state_next[3]  # np.sqrt(state_next[3] ** 2 + state_next[4] ** 2)
    dist_ahead = v * (param.N * param.Ts)  # = velocity * prediction horizon in seconds (cg.N * cg.Ts)
    print("DIST AHEAD = " + str(dist_ahead) + ", v = " + str(v))
    [i_nearest, nearest_dist] = get_nearest_point([state_next[0], state_next[1]], track_x, track_y, param.track_width,
                                                  300, i_nearest)
    [i_ahead, end_of_track_reached] = move_along_track(track_x, track_y, dist_ahead, i_nearest)
    # Find the angle in the direction of the previous step - should this be from current position instead?
    [x, y] = [track_x[i_ahead], track_y[i_ahead]]
    # The next reference state, taking into account all the previous calculations, and using the vehicle
    # model with regard to the obtained control inputs
    state_ref = cg.kinetic_model_rk(state_ref, first_control_input, False)

    return end_of_track_reached, i_nearest, state_ref, i_ahead, x, y


def get_nearest_point(current_pos, track_x, track_y, track_width, search_region, prev_closest):
    # Inspired here: https://github.com/alexliniger/MPCC/blob/master/Matlab/findTheta.m
    # Find the index of the point on the centreline which is the closest to the current position

    # Investigate at search region
    back = int(search_region * 0.5)  # 50%
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
