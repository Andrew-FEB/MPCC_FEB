from warnings import warn
import numpy as np
from collections import namedtuple
from scipy import interpolate
import opengen as og
import codegenerator as cg
import parameters as param

# Author: Darina Abaffyová
# Created: 13/02/2020
# Last updated: 03/04/2020

# Define all named tuples here
KinematicState = namedtuple('KinematicState', 'x y psi v')
DynamicState = namedtuple('DynamicState', 'x y phi v_x v_y omega')
Control = namedtuple('Control', 'D delta')
Intercept = namedtuple('intercept', 'centre upper lower')
Slope = namedtuple('slope', 'centre upper lower')


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
    # the same (tangential) velocity for the next cg.N time steps, with
    # each time step being cg.Ts second long
    dist_ahead = state_0.v * param.N * param.Ts
    # i_nearest is the index of the nearest point on the reference line
    i_nearest, dummy = get_nearest_point((state_0.x, state_0.y), track_x, track_y, 300, 0)
    i_ahead, dummy = move_along_track(track_x, track_y, dist_ahead, i_nearest)
    ref_state = get_reference(i_nearest, i_ahead, track_x, track_y)
    ref_seq = [ref_state]

    cost_seq = [0]

    # The nearest sequence will contain tuples (x, y) of the positions
    # on the reference line which correspond to the nearest point found
    # for each state in the state sequence
    intercept, slope = get_boundaries(i_ahead, i_nearest, lower, track_x, track_y, upper)
    bound_seq = [(slope, intercept, i_nearest, i_ahead)]

    control_inputs = [0] * param.N * param.nu  # warm_start(state, ref_state, [slope, x_nearest, y_nearest])
    control_inputs_seq = []

    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("mpcc_python_build_2/mpcc_optimizer")
    # Start the TCP server
    mng.start()

    # Run simulation
    for k in range(simulation_steps):
        solver_status = mng.call(np.concatenate((state, slope, intercept, np.reshape(ref_state, 2 * param.N))),
                                 control_inputs)

        try:
            print('Loop [' + str(k) + ']: ' + str(solver_status['solve_time_ms']) + ' ms. Exit status: '
                  + solver_status['exit_status'] + '. Outer iterations: ' + str(solver_status['num_outer_iterations'])
                  + '. Inner iterations: ' + str(solver_status['num_inner_iterations'])
                  + '. Penalty: ' + str(solver_status['penalty'])
                  + '. Ref index = ' + str(i_ahead) + '. Nearest index = ' + str(i_nearest))

            control_inputs = solver_status['solution']
            first_control_input = Control(control_inputs[0], control_inputs[1])
            state_next = cg.kinematic_model_rk(state, first_control_input, False)

            end_of_track_reached, i_nearest, ref_state, i_ahead = update_reference(first_control_input, i_nearest,
                                                                                   state_next, ref_state, track_x,
                                                                                   track_y)
            if end_of_track_reached: break

            # Update all variables needed for the next iteration and save values in the sequences to be plotted
            intercept, slope = get_boundaries(i_ahead, i_nearest, lower, track_x, track_y, upper)

            # ref_state = get_reference(i_nearest, i_ahead, track_x, track_y)
            state = KinematicState(state_next[0], state_next[1], state_next[2], state_next[3])
            first_control_seq.append(first_control_input)
            state_seq.append(state)
            ref_seq.append(ref_state)
            bound_seq.append((slope, intercept, i_nearest, i_ahead))
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


def get_boundaries(i_ahead, i_nearest, lower, track_x, track_y, upper):
    i_next = i_nearest + 1

    # i_nearest = i_nearest + round((i_ahead - i_nearest)/2)
    x_nearest = track_x[i_nearest]
    x_nearest_next = track_x[len(track_x) - 1 if i_nearest + 1 >= len(track_x) else i_nearest + 1]

    # Centre
    y_nearest = track_y[i_nearest]
    y_nearest_next = track_y[len(track_y) - 1 if i_nearest + 1 >= len(track_y) else i_nearest + 1]
    slope = (y_nearest_next - y_nearest) / (x_nearest_next - x_nearest)
    # 0 if math.isnan((y_nearest_next - y_nearest) / (x_nearest_next - x_nearest))\
    #     else (y_nearest_next - y_nearest) / (x_nearest_next - x_nearest)
    # y = slope * x + y_intercept
    intercept = y_nearest - slope * x_nearest
    x_nearest_next = track_x[i_next]

    # Upper boundary
    y_up = upper[i_nearest]
    y_up_next = upper[i_next]  # upper[len(upper) - 1 if i_nearest + 1 >= len(upper) else i_nearest + 1]
    slope_up = (y_up_next - y_up) / (x_nearest_next - x_nearest)
    # 0 if math.isnan((y_up_next - y_up) / (x_nearest_next - x_nearest))\
    # else (y_up_next - y_up) / (x_nearest_next - x_nearest)
    # y = slope * x + y_intercept
    intercept_up = y_up - slope_up * x_nearest

    # Lower boundary
    y_low = lower[i_nearest]
    y_low_next = lower[i_next]  # lower[len(lower) - 1 if i_nearest + 1 >= len(lower) else i_nearest + 1]
    slope_low = (y_low_next - y_low) / (x_nearest_next - x_nearest)
    # 0 if math.isnan((y_low_next - y_low) / (x_nearest_next - x_nearest))\
    # else (y_low_next - y_low) / (x_nearest_next - x_nearest)
    # y = slope * x + y_intercept
    intercept_low = y_low - slope_low * x_nearest

    intercepts = Intercept(intercept, intercept_up, intercept_low)
    slopes = Slope(slope, slope_up, slope_low)

    return intercepts, slopes


def get_reference(i_nearest, i_ahead, track_x, track_y):
    x = np.array([track_x[i_nearest], track_x[round(i_nearest + (i_ahead - i_nearest)/4)],
                  track_x[round(i_nearest + (i_ahead - i_nearest)/2)], track_x[i_ahead]])
    y = np.array([track_y[i_nearest], track_y[round(i_nearest + (i_ahead - i_nearest)/4)],
                  track_y[round(i_nearest + (i_ahead - i_nearest)/2)], track_y[i_ahead]])

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x, y], s=0, per=False)

    # evaluate the spline fits for 1000 evenly spaced distance values
    x, y = interpolate.splev(np.linspace(0, 1, param.N), tck)

    return list(zip(x, y))


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
    v = state_next[3]
    dist_ahead = v * (param.N * param.Ts)  # = velocity * prediction horizon in seconds (cg.N * cg.Ts)
    print("DIST AHEAD = " + str(dist_ahead) + ", v = " + str(v))
    i_nearest, nearest_dist = get_nearest_point([state_next[0], state_next[1]], track_x, track_y, 300, i_nearest)
    i_ahead, end_of_track_reached = move_along_track(track_x, track_y, dist_ahead, i_nearest)

    # The next reference state, taking into account all the previous calculations, and using the vehicle
    # model with regard to the obtained control inputs
    state_ref = get_reference(i_nearest, i_ahead, track_x, track_y)
    # cg.kinematic_model_rk(state_ref, first_control_input, False)

    return end_of_track_reached, i_nearest, state_ref, i_ahead


def get_nearest_point(current_pos, track_x, track_y, search_region, prev_closest):
    # Inspired here: https://github.com/alexliniger/MPCC/blob/master/Matlab/findTheta.m
    # Find the index of the point on the centreline which is the closest to the current position
    track_width = param.track_width

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
