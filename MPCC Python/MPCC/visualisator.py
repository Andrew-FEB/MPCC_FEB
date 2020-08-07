import time

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

import codegenerator as cg
import parameters as param


# Author: Darina Abaffyová
# Created: 16/05/2020

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


def plot_track(track_x, track_y, left_x, left_y, right_x, right_y, state_seq):
    state_x = [x for x, *_ in state_seq]
    state_y = [y for x, y, *_ in state_seq]
    fig, ax = plt.subplots(1, 1)

    ax.plot(track_x, track_y, '.y', label="Complete track")
    ax.plot(left_x, left_y, '--g', label="Boundaries")
    ax.plot(right_x, right_y, '--g')
    ax.plot(state_x, state_y, 'or', label="Achieved track")

    plt.grid()
    plt.axis('equal')
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

    for i in range(len(state_x)):
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


def plot_dynamic(track_x, track_y, left_x, left_y, right_x, right_y, state_seq, ref_seq, bound_seq, cost_seq,
                 control_seq, exit_status_seq):
    state_x = [x for x, *_ in state_seq]
    state_y = [y for x, y, *_ in state_seq]
    ref_x = [x for x, *_ in ref_seq[0]]
    ref_y = [y for x, y in ref_seq[0]]
    slopes = [s for s, *_ in bound_seq]
    intercepts = [i for s, i, *_ in bound_seq]
    track_widths = [tw for s, i, tw, *_ in bound_seq]

    plt.show()
    ax = plt.gca()

    ax.plot(track_x, track_y, '.y', label="Complete track", markersize=1)
    achieved_line, = ax.plot(state_x[0], state_y[0], 'or', label="Achieved track")
    ax.plot(left_x, left_y, '--g', label="Complete boundaries", linewidth=1)
    ax.plot(right_x, right_y, '--g', linewidth=1)
    bound_left, = ax.plot(ref_x, [slopes[0].left * x + intercepts[0].left for x in ref_x], 'm--')
    bound_right, = ax.plot(ref_x, [slopes[0].right * x + intercepts[0].right for x in ref_x], 'm--')
    bound_left.set_label("Boundaries for PH")
    ref_line, = ax.plot(ref_x, ref_y, '.c', label="Reference line for PH")
    predicted_line, = ax.plot(track_x[0], track_y[0], '.b', label="Predicted track")

    txt = plt.text(0, 0, '', color="black", fontsize=18)

    plt.grid()
    plt.axis('equal')
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
        # plt.arrow(state[0], state[1], 2, 2 * np.tan(state[3]), ec='black')
        # Calculate positions obtained with these control inputs
        for j in range(0, len(control_seq[i]), param.nu):
            next_state = cg.kinematic_model_rk(state, [control_seq[i][j], control_seq[i][j + 1]], False)
            x.append(next_state[0])
            y.append(next_state[1])
            state = next_state

        achieved_line.set_xdata(state_x[0:i])
        achieved_line.set_ydata(state_y[0:i])

        bound_left.set_xdata([x for x, *_ in ref_seq[i]])
        bound_left.set_ydata([slopes[i].left * x + intercepts[i].left for x, *_ in ref_seq[i]])
        bound_right.set_xdata([x for x, *_ in ref_seq[i]])
        bound_right.set_ydata([slopes[i].right * x + intercepts[i].right for x, *_ in ref_seq[i]])

        predicted_line.set_xdata(x)
        predicted_line.set_ydata(y)
        ref_line.set_xdata([x for x, *_ in ref_seq[i]])
        ref_line.set_ydata([y for x, y in ref_seq[i]])

        txt.set_text("Iteration: " + str(i) + "\nExit status: " + exit_status_seq[i]
                     + "\nTrack width: " + str(round(track_widths[i], 3)))
        plt.draw()
        plt.pause(1e-17)
        time.sleep(1e-7)

    plt.show()


def plot_solve_time(solve_time_seq, exit_status_seq):
    t = np.arange(0, param.Ts * len(solve_time_seq), param.Ts)

    ess = []
    tess = []
    empty = True
    for i in range(len(exit_status_seq)):
        if exit_status_seq[i].startswith("NotConverged"):
            ess.append(solve_time_seq[i])
            tess.append(t[i])
            if empty: empty = False

    avg = np.mean(solve_time_seq)
    std = np.std(solve_time_seq)

    plt.plot(t[0:len(solve_time_seq)], solve_time_seq)
    plt.plot(t[0:len(solve_time_seq)], [avg] * len(solve_time_seq))
    if not empty: plt.plot(tess, ess, '.r', markersize=10)
    plt.text(0, 0, "Avg = " + str(avg) + ", Std = " + str(std))

    plt.grid()
    plt.ylabel('Solve time')
    plt.xlabel('Time step')
    plt.title('Solve time at each iteration')
    plt.show()


def plot_tracks_from_files(track_x, track_y, left_x, left_y, right_x, right_y):
    N40 = open('N40tew0.05.txt', 'r')
    lines = N40.readlines()
    N40.close()
    x_n40 = [float(l.split()[0].replace(',', '')) for l in lines]
    y_n40 = [float(l.split()[1].replace(',', '')) for l in lines]

    N30 = open('N30tew0.05.txt', 'r')
    lines = N30.readlines()
    N30.close()
    x_n30 = [float(l.split()[0].replace(',', '')) for l in lines]
    y_n30 = [float(l.split()[1].replace(',', '')) for l in lines]

    N20 = open('N20tew0.05.txt', 'r')
    lines = N20.readlines()
    N20.close()
    x_n20 = [float(l.split()[0].replace(',', '')) for l in lines]
    y_n20 = [float(l.split()[1].replace(',', '')) for l in lines]

    fig, ax = plt.subplots(1, 1)

    ax.plot(track_x, track_y, '.y', label="Complete track", markersize=1)
    ax.plot(left_x, left_y, '--g', label="Complete boundaries", linewidth=1)
    ax.plot(right_x, right_y, '--g', linewidth=1)
    ax.plot(x_n40, y_n40, label="N = 40", markersize=2)
    ax.plot(x_n30, y_n30, label="N = 30", markersize=2)
    ax.plot(x_n20, y_n20, label="N = 20", markersize=2)

    # tew01 = open('N35tew0.1.txt', 'r')
    # lines = tew01.readlines()
    # tew01.close()
    # x_tew01 = [float(l.split()[0].replace(',', '')) for l in lines]
    # y_tew01 = [float(l.split()[1].replace(',', '')) for l in lines]
    #
    # tew05 = open('N35tew0.5.txt', 'r')
    # lines = tew05.readlines()
    # tew05.close()
    # x_tew05 = [float(l.split()[0].replace(',', '')) for l in lines]
    # y_tew05 = [float(l.split()[1].replace(',', '')) for l in lines]
    #
    # tew1 = open('N35tew1.txt', 'r')
    # lines = tew1.readlines()
    # tew1.close()
    # x_tew1 = [float(l.split()[0].replace(',', '')) for l in lines]
    # y_tew1 = [float(l.split()[1].replace(',', '')) for l in lines]
    #
    # fig, ax = plt.subplots(1, 1)
    #
    # ax.plot(track_x, track_y, '.y', markersize=1)
    # ax.plot(left_x, left_y, '--g', linewidth=1)
    # ax.plot(right_x, right_y, '--g', linewidth=1)
    # ax.plot(x_tew05, y_tew01, label="w1 = 0.1", markersize=2)
    # ax.plot(x_tew05, y_tew05, label="w1 = 0.5", markersize=2)
    # ax.plot(x_tew1, y_tew1, label="w1 = 1", markersize=2)

    plt.axis('equal')
    plt.grid()
    plt.ylabel('Y position')
    plt.xlabel('X position')
    plt.title('Track')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()


def plot_times_from_files():
    N40 = open('times_N40tew0.05.txt', 'r')
    lines = N40.readlines()
    N40.close()
    n40 = [float(l) for l in lines]
    avg_n40 = np.mean(n40)
    std_n40 = np.std(n40)

    N30 = open('times_N30tew0.05.txt', 'r')
    lines = N30.readlines()
    N30.close()
    n30 = [float(l) for l in lines]
    avg_n30 = np.mean(n30)
    std_n30 = np.std(n30)

    N20 = open('times_N20tew0.05.txt', 'r')
    lines = N20.readlines()
    N20.close()
    n20 = [float(l) for l in lines]
    avg_n20 = np.mean(n20)
    std_n20 = np.std(n20)

    p = plt.plot(n40, label="N = 40")
    plt.plot([avg_n40] * len(n40), color=p[0].get_color())

    p = plt.plot(n30, label="N = 30")
    plt.plot([avg_n30] * len(n30), color=p[0].get_color())

    p = plt.plot(n20, label="N = 20")
    plt.plot([avg_n20] * len(n20), color=p[0].get_color())

    plt.text(0, 50, "N = 40: Avg = " + str(round(avg_n40, 3)) + ", Std = " + str(round(std_n40, 3))
             + "\nN = 30: Avg = " + str(round(avg_n30, 3)) + ", Std = " + str(round(std_n30, 3))
             + "\nN = 20: Avg = " + str(round(avg_n20, 3)) + ", Std = " + str(round(std_n20, 3)))

    # tew05 = open('times_N35tew0.5.txt', 'r')
    # lines = tew05.readlines()
    # tew05.close()
    # tew05 = [float(l) for l in lines]
    # avg_tew05 = np.mean(tew05)
    # std_tew05 = np.std(tew05)
    #
    # tew1 = open('times_N35tew1.txt', 'r')
    # lines = tew1.readlines()
    # tew1.close()
    # tew1 = [float(l) for l in lines]
    # avg_tew1 = np.mean(tew1)
    # std_tew1 = np.std(tew1)
    #
    # p = plt.plot(tew05, label="w1 = 0.5")
    # plt.plot([avg_tew05] * len(tew05), color=p[0].get_color())
    #
    # p = plt.plot(tew1, label="w1 = 1")
    # plt.plot([avg_tew1] * len(tew1), color=p[0].get_color())
    #
    # plt.text(0, 50, "w1 = 0.5: Avg = " + str(round(avg_tew05, 3)) + ", Std = " + str(round(std_tew05, 3))
    #          + "\nw1 = 1: Avg = " + str(round(avg_tew1, 3)) + ", Std = " + str(round(std_tew1, 3)))

    plt.grid()
    plt.ylabel('Solve time')
    plt.xlabel('Time step index')
    plt.title('Solve time at each iteration')
    plt.legend(loc='best', borderaxespad=0.)
    plt.show()
