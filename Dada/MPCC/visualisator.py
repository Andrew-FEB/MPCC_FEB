import time

import matplotlib.pyplot as plt
import numpy as np

import codegenerator as cg
import parameters as param


# Author: Darina Abaffyová
# Created: 16/05/2020
# Last updated: 30/05/2020

def plot_cost(cost_seq):
    t = np.arange(0, param.Ts * len(cost_seq), param.Ts)
    plt.plot(t[0:len(cost_seq)], cost_seq)
    plt.grid()
    plt.ylabel('Cost')
    plt.xlabel('Time')
    plt.title('Cost in each iteration')
    plt.show()


def plot_simulation(simulation_steps, input_seq, state_seq, ref_seq):
    t = np.arange(0, param.Ts * (simulation_steps - param.Ts), param.Ts)

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


def plot_dynamic(track_x, track_y, upper, lower, state_seq, ref_seq, bound_seq, cost_seq, control_seq):
    state_x = [x for x, *_ in state_seq]
    state_y = [y for x, y, *_ in state_seq]
    ref_x = [x for x, *_ in ref_seq[0]]
    ref_y = [y for x, y in ref_seq[0]]
    slopes = [s for s, *_ in bound_seq]
    intercepts = [i for s, i, *_ in bound_seq]
    index1 = [index for s, i, index in bound_seq]
    index2 = [index + 10 for s, i, index in bound_seq]

    plt.show()
    ax = plt.gca()

    ax.plot(track_x, track_y, '.y', label="Complete track", markersize=1)
    achieved_line, = ax.plot(state_x[0], state_y[0], 'or', label="Achieved track")
    ax.plot(track_x, upper, '--g', label="Complete boundaries", linewidth=1)
    ax.plot(track_x, lower, '--g', linewidth=1)
    predicted_line, = ax.plot(track_x[0], track_y[0], '.b', label="Predicted track")
    ref_line, = ax.plot(ref_x, ref_y, '.c', label="Reference line for PH")

    bound_up = [0] * param.N
    bound_low = [0] * param.N
    for j in range(param.N):
        bound_up[j], = ax.plot(track_x[index1[0][j]:index2[0][j]],
                               [slopes[0][j].upper * x + intercepts[0][j].upper for x in
                                track_x[index1[0][j]:index2[0][j]]], 'm--')
        bound_low[j], = ax.plot(track_x[index1[0][j]:index2[0][j]],
                                [slopes[0][j].lower * x + intercepts[0][j].lower
                                 for x in track_x[index1[0][j]:index2[0][j]]], 'm--')
    # txt = plt.text(5, 0, '', color="black", fontsize=12)

    bound_up[0].set_label("Boundaries for PH")
    plt.grid()
    plt.ylabel('Y position')
    plt.xlabel('X position')
    plt.title('Track')
    plt.legend(loc='best', borderaxespad=0.)

    plt.pause(1e-17)
    time.sleep(3)

    for i in range(len(control_seq)):
        state = state_seq[i]
        x = [state[0]]
        y = [state[1]]
        # Calculate positions obtained with these control inputs
        for j in range(0, len(control_seq[i]), param.nu):
            next_state = cg.kinematic_model_rk(state, [control_seq[i][j], control_seq[i][j + 1]], False)
            x.append(next_state[0])
            y.append(next_state[1])
            state = next_state

        achieved_line.set_xdata(state_x[0:i])
        achieved_line.set_ydata(state_y[0:i])

        for j in range(param.N):
            bound_up[j].set_xdata(track_x[index1[i][j]:index2[i][j]])
            bound_up[j].set_ydata(
                [slopes[i][j].upper * x + intercepts[i][j].upper for x in track_x[index1[i][j]:index2[i][j]]])
            bound_low[j].set_xdata(track_x[index1[i][j]:index2[i][j]])
            bound_low[j].set_ydata(
                [slopes[i][j].lower * x + intercepts[i][j].lower for x in track_x[index1[i][j]:index2[i][j]]])

        ref_line.set_xdata([x for x, *_ in ref_seq[i]])
        ref_line.set_ydata([y for x, y in ref_seq[i]])
        predicted_line.set_xdata(x)
        predicted_line.set_ydata(y)
        # dist_centre = round((abs((slopes[i].centre * state_x[i] - state_y[i] + intercepts[i].centre)) /
        #                      (np.sqrt(slopes[i].centre ** 2 + 1))), 3)
        # dist_up = round((abs((slopes[i].upper * state_x[i] - state_y[i] + intercepts[i].upper)) /
        #                  (np.sqrt(slopes[i].upper ** 2 + 1))), 3)
        # dist_low = round((abs((slopes[i].lower * state_x[i] - state_y[i] + intercepts[i].lower)) /
        #                   (np.sqrt(slopes[i].lower ** 2 + 1))), 3)
        # txt.set_text('Cost = ' + str(round(cost_seq[i], 3)) + '\nCentre line distance = ' + str(dist_centre)
        #              + '\nUpper bound distance = ' + str(dist_up) + '\nLower bound distance = '
        #              + str(dist_low) + '\n' + str(state_seq[i]))
        plt.draw()
        plt.pause(1e-17)
        time.sleep(1e-5)

    plt.show()
