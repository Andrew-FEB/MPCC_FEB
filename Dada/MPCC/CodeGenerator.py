import casadi.casadi as cs
import opengen as og
import numpy as np

# Author: Darina Abaffyová
# Created: 12/02/2020
# Last updated: 29/02/2020

# Parameters
# -------------------------------------
# Vehicle parameters
# I_z = 7  # [kg m^2] moment of inertia of the vehicle ASK FOR THIS VALUE!!!!!!!
# m = 208  # [kg] mass of the vehicle
# l_f = 0.845  # [m] length of the front part of the vehicle (this is from front axle to COG)
# l_r = 0.690  # [m] length of the rear part of the vehicle  (this is from front axle to COG)
m = 1573
I_z = 2873
l_f = 1.35
l_r = 1.35

wight_f = l_r / (l_f + l_r)
wight_r = l_f / (l_f + l_r)

C_m1 = 17303
C_m2 = 175
C_rr = 120
C_d = 0.5 * 1.225 * 0.35 * 2.5

B_r = 13
C_r = 2
D_r = wight_f * m * 9.81 * 1.2

B_f = 13
C_f = 2
D_f = wight_r * m * 9.81 * 1.2

# L = 5;
# W = 2.5;

# Tire Parameters (TBD)
# Tire Force Curve
# B_f = 1
# B_r = 1
# C_f = 1
# C_r = 1
# D_f = 1
# D_r = 1

# Motor Model
# C_m1 = 1
# C_m2 = 1

# Rolling Resistance
# C_rr = 1

# Drag
# C_d = 1

N = 40  # Prediction Horizon (in time steps)
nu = 2  # Number of Decision Variables (input)
nx = 6  # Number of Parameters (state)
Ts = 0.05  # 0.05  # Sampling time (length of one time step)


# Model
# -------------------------------------
def dynamic_model_ct(state, control, tire_f):
    # State variables
    phi = state[2]
    v_x = state[3]
    v_y = state[4]
    omega = state[5]
    # Control variables
    delta = control[1]
    # Tire forces
    f_fy = tire_f[0]
    f_rx = tire_f[1]
    f_ry = tire_f[2]

    x = v_x * cs.cos(phi) - v_y * cs.sin(phi)
    y = v_x * cs.sin(phi) + v_y * cs.cos(phi)
    phi = omega
    v_x = 1 / m * (f_rx - f_fy * cs.sin(delta) + m * v_y * omega)
    v_y = 1 / m * (f_ry - f_fy * cs.cos(delta) - m * v_x * omega)
    omega = 1 / I_z * (f_fy * l_f * cs.cos(delta) - f_ry * l_r)

    return [x, y, phi, v_x, v_y, omega]


def dynamic_model(state, control, forces, dt):
    # State variables
    x = state[0]
    y = state[1]
    phi = state[2]
    v_x = state[3]
    v_y = state[4]
    omega = state[5]
    # Control variables
    delta = control[1]
    # Tire forces
    f_fy = forces[0]
    f_rx = forces[1]
    f_ry = forces[2]

    x_next = x + dt * (v_x * cs.cos(phi) - v_y * cs.sin(phi))
    y_next = y + dt * (v_x * cs.sin(phi) + v_y * cs.cos(phi))
    phi_next = phi + dt * omega
    v_x_next = v_x + dt * (1 / m * (f_rx - f_fy * cs.sin(delta) + m * v_y * omega))
    v_y_next = v_y + dt * (1 / m * (f_ry - f_fy * cs.cos(delta) - m * v_x * omega))
    omega_next = omega + dt * (1 / I_z * (f_fy * l_f * cs.cos(delta) - f_ry * l_r))

    return [x_next, y_next, phi_next, v_x_next, v_y_next, omega_next]


def dynamic_model_dt(state, control, forces, dt):
    [x, y, phi, v_x, v_y, omega] = dynamic_model_ct(state, control, forces)
    return state + cs.vertcat(dt * x, dt * y, dt * phi, dt * v_x, dt * v_y, dt * omega, state[6], state[7], state[8], state[9], state[10], state[11])


# def runge_kutta(state, control, forces, dt):
#
#
#
#   # Update next state
# next_state = [0] * nx
# for i in range(0, nx):
# next_state[i] += state[i] + (1.0 / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
#
# return next_state


def tire_forces(state, control):
    # State variables
    v_x = state[3]
    v_y = state[4]
    omega = state[5]
    # Control variables
    D = control[0]
    delta = control[1]

    # Force calculations

    alpha_f = -cs.atan2((l_f * omega + v_y), v_x) + delta
    alpha_r = cs.atan2((l_r * omega - v_y), v_x)

    F_fy = D_f * cs.sin(C_f * cs.atan(B_f * alpha_f))
    F_ry = D_r * cs.sin(C_r * cs.atan(B_r * alpha_r))

    F_rx = (C_m1 * D - C_m2 * D * v_x - C_rr - C_d * v_x ** 2)

    # ADD THE TIRE CONSTRAINT (friction ellipse)

    return [F_fy, F_rx, F_ry]


def tire_forces_dt(forces, state, control, dt):
    [F_fy, F_rx, F_ry] = tire_forces(state, control)
    return forces + [F_fy * dt, F_rx * dt, F_ry * dt]  # forces + dt * cs.vertcat(F_fy, F_rx, F_ry)


# Cost function:
def cost_function(x, u, u_prev, state_error_weight, d_change_weight, delta_change_weight):
    # Cost on state error
    cf = 0
    for i in range(0, nx):
        cf += state_error_weight * (x[i] - x[nx + i]) ** 2
    # Cost on input change
    cf += d_change_weight * (u_prev[0] - u[0]) ** 2
    cf += delta_change_weight * (u_prev[1] - u[1]) ** 2
    return cf


# Problem
# -------------------------------------
def generate_code(state_error_weight, d_change_weight, delta_change_weight):
    # Not sure about the u_seq - should it be nu*N large ???
    u_seq = cs.MX.sym("u", nu * N)  # Sequence of all inputs
    x0 = cs.MX.sym("x0_xref", nx * 2)  # Initial state + Reference state

    cost = 0
    x_t = x0
    f = tire_forces(x0, [u_seq[0], u_seq[1]])
    F1 = []
    F2 = []
    for t in range(0, nu * N, nu):
        if t >= 2:
            u_prev = [u_seq[t - 2], u_seq[t - 1]]
        else:
            u_prev = [0, 0]

        u = [u_seq[t], u_seq[t + 1]]
        cost += cost_function(x_t, u, u_prev, state_error_weight, d_change_weight, delta_change_weight)  # Update cost
        f = tire_forces_dt(f, x_t, u, Ts)
        x_t = dynamic_model_dt(x_t, u, f, Ts)  # Update state

        F1 = cs.vertcat(F1, x_t[0], x_t[1], x_t[2], x_t[3], x_t[4], x_t[5])
        F2 = cs.vertcat(F2, cs.fmax(cs.fabs(u[0] - u_prev[0]), 0.001),
                        cs.fmax(cs.fabs(u[1] - u_prev[1]), 0.001))

    # Constraints
    # -------------------------------------
    U = og.constraints.BallInf(None, 0.75)
    # C = og.constraints.Rectangle([-7, -7, -0.75, -0.5], [7, 7, 0.75, 0.5])
    # U = og.constraints.Rectangle([-0.95, -0.5], [0.95, 0.5])
    C = og.constraints.Rectangle([-7, -7, -0.95, -150, -150, -100], [7, 7, 0.75, 150, 150, 100])
    # C = og.constraints.BallInf(None, 0.95)

    # Code Generation
    # -------------------------------------
    problem = og.builder.Problem(u_seq, x0, cost) \
        .with_constraints(U) \
        .with_aug_lagrangian_constraints(F1, C) \
        .with_penalty_constraints(F2)

    build_config = og.config.BuildConfiguration() \
        .with_build_directory("mpcc_python_build_1") \
        .with_tcp_interface_config()

    meta = og.config.OptimizerMeta().with_optimizer_name("mpcc_optimizer") \
        .with_authors("Darina Abaffyova") \
        .with_version("1.0.0")

    solver_config = og.config.SolverConfiguration() \
        .with_tolerance(1e-7) \
        .with_initial_tolerance(1e-7) \
        .with_max_outer_iterations(100) \
        .with_max_inner_iterations(200) \
        .with_delta_tolerance(1e-5) \
        .with_initial_penalty(30) \
        .with_penalty_weight_update_factor(15.0)

    builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                              build_config, solver_config)
    builder.build()
