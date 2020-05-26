import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import time
import parameters as param
import codegenerator as cg


# Author: Darina Abaffyová
# Created: 16/05/2020
# Last updated: 16/05/2020

def plot_cost(cost_seq):
    t = np.arange(0, param.Ts * len(cost_seq), param.Ts)
    plt.plot(t[0:len(cost_seq)], cost_seq)
    plt.grid()
    plt.ylabel('Cost')
    plt.xlabel('Time')
    plt.title('Cost in each iteration')
    plt.show()


def plot_simulation(simulation_steps, input_seq, state_seq, ref_seq):
    t = np.arange(0, param.Ts * (simulation_steps - param.Ts), param.Ts)  # - cg.Ts

    plt.plot(t, [D for D, delta in input_seq], '-', label="Throttle")
    plt.plot(t, [delta for D, delta in input_seq], '-', label="Front steering angle")
    plt.grid()
    plt.ylabel('Input')
    plt.xlabel('Time')
    plt.title('INPUT SEQUENCE')
    plt.legend(bbox_to_anchor=(0.7, 0.85), loc='best', borderaxespad=0.)
    plt.show()

    plt.plot(t, [x for x, *_ in state_seq], '-', label="x")
    plt.plot(t, [y for x, y, *_ in state_seq], '-', label="y")
    plt.plot(t, [phi for x, y, phi, *_ in state_seq], '-', label="phi")
    plt.plot(t, [x for x, *_ in ref_seq], '--', label="x")
    plt.plot(t, [y for x, y, *_ in ref_seq], '--', label="y")
    plt.plot(t, [phi for x, y, phi, *_ in ref_seq], '--', label="phi")
    plt.grid()
    plt.ylabel('States')
    plt.xlabel('Time')
    plt.title('STATE SEQUENCE 1')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()

    plt.plot(t, [v_x for x, y, phi, v_x, *_ in state_seq], '-', label="v_x")
    plt.plot(t, [v_y for x, y, phi, v_x, v_y, omega in state_seq], '-', label="v_y")
    plt.plot(t, [omega for x, y, phi, v_x, vy, omega in state_seq], '-', label="omega")
    plt.plot(t, [v_x for x, y, phi, v_x, *_ in ref_seq], '--', label="v_x")
    plt.plot(t, [v_y for x, y, phi, v_x, v_y, omega in ref_seq], '--', label="v_y")
    plt.plot(t, [omega for x, y, phi, v_x, vy, omega in ref_seq], '--', label="omega")
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


def plot_track_with_cost_values(track_x, track_y, state_seq, cost_seq, nearest_seq):
    state_x = [x for x, *_ in state_seq]
    state_y = [y for x, y, *_ in state_seq]
    nearest_x = [x for x, y, *_ in nearest_seq]
    nearest_y = [y for x, y, *_ in nearest_seq]

    fig, ax = plt.subplots(1, 1)
    ax.plot(track_x, track_y, '.y', label="Complete track")
    ax.plot(state_x, state_y, 'or', label="Achieved track")

    i = 0
    for x, y in zip(state_x, state_y):
        ax.plot([nearest_x[i], state_x[i]], [nearest_y[i], state_y[i]], '-k')
        plt.text(x - 0.05, y - 0.05, str(round(cost_seq[i], 1)), color="blue", fontsize=10)
        i += 1

    plt.axis('equal')
    plt.grid()
    plt.ylabel('Y position')
    plt.xlabel('X position')
    plt.title('Track')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()


def plot_dynamic(track_x, track_y, upper, lower, state_seq, ref_seq, nearest_seq, cost_seq, control_seq):
    state_x = [x for x, *_ in state_seq]
    state_y = [y for x, y, *_ in state_seq]
    ref_x = [x for x, *_ in ref_seq]
    ref_y = [y for x, y, *_ in ref_seq]
    slope_up = [s1 for s1, i1, s2, i2, index1, index2 in nearest_seq]
    intercept_up = [i1 for s1, i1, s2, i2, index1, index2 in nearest_seq]
    slope_low = [s2 for s1, i1, s2, i2, index1, index2 in nearest_seq]
    intercept_low = [i2 for s1, i1, s2, i2, index1, index2 in nearest_seq]
    index1 = [index1 for s1, i1, s2, i2, index1, index2 in nearest_seq]
    index2 = [index2 for s1, i1, s2, i2, index1, index2 in nearest_seq]

    plt.show()
    ax = plt.gca()

    ax.plot(track_x, track_y, '.y', label="Complete track")
    achieved_line, = ax.plot(state_x[0], state_y[0], 'or', label="Achieved track")
    ax.plot(track_x, upper, '--g', label="Boundaries")
    ax.plot(track_x, lower, '--g')
    predicted_line, = ax.plot(track_x[0], track_y[0], 'b.', label="Predicted track")
    ref_point, = ax.plot(ref_x[0], ref_y[0], 'rx', label="Reference point", markersize=10)
    bound_up, = ax.plot(track_x[index1[0]:index2[0]],
                        [slope_up[0] * x + intercept_up[0] for x in track_x[index1[0]:index2[0]]],
                        'm--', label="Upper tangent line", markersize=10)
    bound_low, = ax.plot(track_x[index1[0]:index2[0]], [slope_low[0] * x + intercept_low[0]
                                                        for x in track_x[index1[0]:index2[0]]],
                         'c--', label="Lower tangent line", markersize=10)
    txt = plt.text(5, 13, '', color="black", fontsize=12)

    plt.axis('equal')
    plt.grid()
    plt.ylabel('Y position')
    plt.xlabel('X position')
    plt.title('Track')
    plt.legend(loc='best', borderaxespad=0.)

    plt.pause(1e-17)
    time.sleep(1)

    for i in range(len(control_seq)):
        state = state_seq[i]
        x = [state[0]]
        y = [state[1]]
        # Calculate positions obtain with these control inputs
        for j in range(0, len(control_seq[i]), param.nu):
            next_state = cg.kinematic_model_rk(state, [control_seq[i][j], control_seq[i][j + 1]], False)
            x.append(next_state[0])
            y.append(next_state[1])
            state = next_state

        achieved_line.set_xdata(state_x[0:i])
        achieved_line.set_ydata(state_y[0:i])
        bound_up.set_xdata(track_x[index1[i]:index2[i]])
        bound_up.set_ydata([slope_up[i] * x + intercept_up[i] for x in track_x[index1[i]:index2[i]]])
        bound_low.set_xdata(track_x[index1[i]:index2[i]])
        bound_low.set_ydata([slope_low[i] * x + intercept_low[i] for x in track_x[index1[i]:index2[i]]])
        ref_point.set_xdata(ref_x[i])
        ref_point.set_ydata(ref_y[i])
        predicted_line.set_xdata(x)
        predicted_line.set_ydata(y)
        dist1 = round(np.sqrt((state_x[i] - ref_x[i]) ** 2 + (state_y[i] - ref_y[i]) ** 2), 3)
        # dist2 = round((abs((slope[i] * state_x[i] - state_y[i] + intercept[i])) / (np.sqrt(slope[i] ** 2 + 1))), 3)
        txt.set_text('Cost = ' + str(round(cost_seq[i], 3)) + '\nReference distance = ' + str(dist1) +
                     '\n[x, y, omega, v] = ' + str(state_seq[i]))  # '\nLine distance = ' + str(dist2) +
        plt.draw()
        plt.pause(1e-17)
        time.sleep(0.005)

    plt.show()
