import casadi.casadi as cs
import opengen as og
import numpy as np

# Author: Darina Abaffyová
# Created: 12/02/2020
# Last updated: 03/04/2020

# Parameters
# -------------------------------------
# Vehicle parameters TODO - get all these
# I_z = 7  # [kg m^2] moment of inertia of the vehicle ASK FOR THIS VALUE!!!!!!!
# m = 208  # [kg] mass of the vehicle
# l_f = 0.845  # [m] length of the front part of the vehicle (this is from front axle to COG)
# l_r = 0.690  # [m] length of the rear part of the vehicle  (this is from front axle to COG)
# I_z = 2873
m = 1573
I_z = 2873
l_f = 1.35
l_r = 1.35

wight_f = l_r / (l_f + l_r)
wight_r = l_f / (l_f + l_r)

# TODO Tire Parameters (TBD) - THESE STILL NEED TO BE CHANGED TO THE ONES CORRESPONDING TO THE FORMULA
C_m1 = 17303  # Motor Model
C_m2 = 175  # Motor Model
C_rr = 120  # Rolling Resistance
C_d = 0.5 * 1.225 * 0.35 * 2.5  # Drag

# Tire Force Curve
B_r = 13
C_r = 2
D_r = wight_f * m * 9.81 * 1.2

B_f = 13
C_f = 2
D_f = wight_r * m * 9.81 * 1.2

# Friction ellipse
p_long = 0.9
p_ellipse = 0.95

# TODO Model limits (TBD) - THESE STILL NEED TO BE CHANGED TO THE ONES CORRESPONDING TO THE FORMULA
x_max = 30
x_min = -x_max
y_max = 30
y_min = -y_max
phi_max = 10
phi_min = -phi_max
v_x_max = 3
v_x_min = 0.03
v_y_max = 3
v_y_min = -v_y_max
omega_max = 8
omega_min = -omega_max
# Control limits
d_max = 1
d_min = -d_max
delta_max = 0.506  # [rad] =  29 degrees
delta_min = -delta_max

# Track parameters
track_width = 3

# Optimizer parameters
N = 40  # Prediction Horizon (in time steps)
nu = 2  # Number of Decision Variables (input)
nx = 4  # 6  # Number of Parameters (state)
Ts = 0.05  # Sampling time (length of one time step in seconds)


# Model
# -------------------------------------
def kinetic_model(state, v_x_prev, delta, delta_prev, F_x, calc_casadi, dt):
    # State variables
    phi = state[2]
    v_x = state[3]
    v_y = state[4]
    omega = state[5]

    d_delta = (delta - delta_prev) / dt
    d_v_x = (v_x - v_x_prev) / dt

    x_next = v_x * cs.cos(phi) - v_y * cs.sin(phi)
    y_next = v_x * cs.sin(phi) + v_y * cs.cos(phi)
    phi_next = omega
    v_x_next = F_x / m
    v_y_next = (d_delta * v_x + delta * d_v_x) * l_r / (l_r + l_f)
    omega_next = (d_delta * v_x + delta * d_v_x) / (l_r + l_f)

    if calc_casadi:
        return cs.vertcat(x_next, y_next, phi_next, v_x_next, v_y_next, omega_next,
                          state[6], state[7], state[8], state[9], state[10], state[11])
    else:
        return (x_next, y_next, phi_next, v_x_next, v_y_next, omega_next,
                state[6], state[7], state[8], state[9], state[10], state[11])


def kinetic_model_temp(state, control, calc_casadi):
    # First simple (kinematic) model (source: https://github.com/MPC-Berkeley/barc/wiki/Car-Model):
    # get states / inputs
    x = state[0]  # Longitudinal position
    y = state[1]  # Lateral Position
    psi = state[2]  # Yaw rate
    v = state[3]  # Velocity
    a = control[0]  # Acceleration
    d_f = control[1]  # Front steering angle

    # compute slip angle
    beta = cs.arctan2(l_r * cs.tan(d_f), (l_f + l_r))

    # compute next state
    x_next = v * cs.cos(psi + beta)
    y_next = v * cs.sin(psi + beta)
    psi_next = v / l_r * cs.sin(beta)
    v_next = a

    if calc_casadi:
        return cs.vertcat(x_next, y_next, psi_next, v_next,
                          state[4], state[5], state[6], state[7])
    else:
        return (x_next, y_next, psi_next, v_next,
                state[4], state[5], state[6], state[7])


# Runge-Kutta 4th order method
def kinetic_model_rk(state, control, dt, calc_casadi):
    if calc_casadi:
        k1 = dt * kinetic_model_temp(state, control, calc_casadi)
        k2 = dt * kinetic_model_temp(state + dt * 0.5 * k1, control, calc_casadi)
        k3 = dt * kinetic_model_temp(state + dt * 0.5 * k2, control, calc_casadi)
        k4 = dt * kinetic_model_temp(state + dt * k3, control, calc_casadi)
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return cs.vertcat(next_state[0], next_state[1], next_state[2], next_state[3],
                          state[4], state[5], state[6], state[7])
    else:
        k1 = dt * np.array(kinetic_model_temp(state, control, calc_casadi))
        k2 = dt * np.array(kinetic_model_temp(state + dt * 0.5 * k1, control, calc_casadi))
        k3 = dt * np.array(kinetic_model_temp(state + dt * 0.5 * k2, control, calc_casadi))
        k4 = dt * np.array(kinetic_model_temp(state + dt * k3, control, calc_casadi))
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return (next_state[0], next_state[1], next_state[2], next_state[3],
                state[4], state[5], state[6], state[7])


def dynamic_model(state, control, forces, calc_casadi):
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

    x_next = v_x * cs.cos(phi) - v_y * cs.sin(phi)
    y_next = v_x * cs.sin(phi) + v_y * cs.cos(phi)
    phi_next = omega
    v_x_next = 1 / m * (f_rx - f_fy * cs.sin(delta) + m * v_y * omega)
    v_y_next = 1 / m * (f_ry + f_fy * cs.cos(delta) - m * v_x * omega)
    omega_next = 1 / I_z * (f_fy * l_f * cs.cos(delta) - f_ry * l_r)

    if calc_casadi:
        return cs.vertcat(x_next, y_next, phi_next, v_x_next, v_y_next, omega_next,
                          state[6], state[7], state[8], state[9], state[10], state[11], state[12], state[13], state[14])
    else:
        return (x_next, y_next, phi_next, v_x_next, v_y_next, omega_next,
                state[6], state[7], state[8], state[9], state[10], state[11], state[12], state[13], state[14])


# Runge-Kutta 4th order method
def dynamic_model_rk(state, control, forces, dt, calc_casadi):
    if calc_casadi:
        k1 = dt * dynamic_model(state, control, forces, calc_casadi)
        k2 = dt * dynamic_model(state + dt * 0.5 * k1, control, forces, calc_casadi)
        k3 = dt * dynamic_model(state + dt * 0.5 * k2, control, forces, calc_casadi)
        k4 = dt * dynamic_model(state + dt * k3, control, forces, calc_casadi)
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return cs.vertcat(next_state[0], next_state[1], next_state[2], next_state[3], next_state[4], next_state[5],
                          state[6], state[7], state[8], state[9], state[10], state[11], state[12], state[13], state[14])
    else:
        k1 = dt * np.array(dynamic_model(state, control, forces, calc_casadi))
        k2 = dt * np.array(dynamic_model(state + dt * 0.5 * k1, control, forces, calc_casadi))
        k3 = dt * np.array(dynamic_model(state + dt * 0.5 * k2, control, forces, calc_casadi))
        k4 = dt * np.array(dynamic_model(state + dt * k3, control, forces, calc_casadi))
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return (next_state[0], next_state[1], next_state[2], next_state[3], next_state[4], next_state[5],
                state[6], state[7], state[8], state[9], state[10], state[11], state[12], state[13], state[14])


def tire_forces(state, control):
    # State variables
    v_x = state[3]
    v_y = state[4]
    omega = state[5]
    # Control variables
    D = control[0]
    delta = control[1]

    # Force calculations

    alpha_f = - cs.arctan2((l_f * omega + v_y), v_x) + delta
    alpha_r = cs.arctan2((l_r * omega - v_y), v_x)

    F_fy = D_f * cs.sin(C_f * cs.arctan(B_f * alpha_f))
    F_ry = D_r * cs.sin(C_r * cs.arctan(B_r * alpha_r))

    F_rx = (C_m1 - C_m2 * v_x) * D - C_rr - C_d * v_x ** 2

    # TODO Tire constraints

    return [F_fy, F_rx, F_ry]


def cost_function(state, control, control_prev, track_error_weight, in_weight, in_change_weight):
    # TODO - add remaining costs
    cf = 0

    x = state[0]
    y = state[1]
    x_ref = state[4]
    y_ref = state[5]
    slope = state[8]
    x_nearest = state[9]
    y_nearest = state[10]
    # x_ref = state[6]
    # y_ref = state[7]
    # slope = state[12]
    # x_nearest = state[13]
    # y_nearest = state[14]

    # y = slope * x + y_intercept
    y_inter = y_nearest - slope * x_nearest

    # Line: ax + by + c = 0 -> slope * x - y + y_inter = 0
    # Point: (x1, y1) -> (x, y)
    # Distance = (| a*x1 + b*y1 + c |) / (sqrt(a*a + b*b))

    # Contouring Error
    cf += track_error_weight[0] * (cs.fabs((slope * x - y + y_inter)) / (cs.sqrt(slope ** 2 + 1)))

    # Tracking Error
    cf += track_error_weight[1] * cs.sqrt((x - x_ref) ** 2 + (y - y_ref) ** 2)

    # Velocity
    cf -= track_error_weight[2] * cs.fabs(state[3] - state[7])

    # Input Weights
    cf += in_weight[0] * control[0] ** 2
    cf += in_weight[1] * control[1] ** 2

    # Input Change Weights
    cf += in_change_weight[0] * (control[0] - control_prev[0]) ** 2
    cf += in_change_weight[1] * (control[1] - control_prev[1]) ** 2

    return cf


def generate_code(track_error_weight, in_weight, in_change_weight):
    # Not sure about the u_seq - should it be nu*N large ???
    u_seq = cs.MX.sym("u", nu * N)  # Sequence of all inputs
    x0 = cs.MX.sym("x0_xref", nx * 2 + 3 + 6)  # Initial state (=4) + Reference state (=4)
    # + slope (=1) + nearest x, y (=2)
    # + 2x(slope, x, y) for each boundary (=6)

    cost = 0
    x_t = x0[0:8]
    F1 = []
    # F2 = []
    for t in range(0, nu * N, nu):
        u = [u_seq[t], u_seq[t + 1]]
        if t > 1:
            u_prev = [u_seq[t - 2], u_seq[t - 1]]
        else:
            u_prev = [0, 0]
        cost += cost_function(cs.vertcat(x_t, x0[8:17]), u, u_prev, track_error_weight, in_weight,
                              in_change_weight)  # Update cost
        # f = tire_forces(x_t, u)
        # x_t = dynamic_model_rk(x_t, u, f, Ts, True)  # Update state
        x_t = kinetic_model_rk(x_t, u, Ts, True)
        # TODO - add the missing constraints
        # Contouring Constraint
        # slope = x_t[8]
        # x_nearest = x_t[9]
        # y_nearest = x_t[10]
        # y_inter = y_nearest - slope * x_nearest
        # c_e = cs.fabs((slope * x_t[0] - x_t[1] + y_inter)) / (cs.sqrt(slope ** 2 + 1))
        # Boundary Constraints
        bound_slope1 = x0[11]
        bound_x1 = x0[12]
        bound_y1 = x0[13]
        bound_inter1 = bound_y1 - bound_slope1 * bound_x1
        bound_slope2 = x0[14]
        bound_x2 = x0[15]
        bound_y2 = x0[16]
        bound_inter2 = bound_y2 - bound_slope2 * bound_x2
        bound_dist1 = (bound_slope1 * x_t[0] - x_t[1] + bound_inter1) / cs.sqrt(bound_slope1 ** 2 + 1)
        bound_dist2 = (bound_slope2 * x_t[0] - x_t[1] + bound_inter2) / cs.sqrt(bound_slope2 ** 2 + 1)

        # F1 = cs.vertcat(F1, x_t[0], x_t[1], x_t[2], x_t[3], x_t[4], x_t[5], u[0], u[1], c_e)
        # F1 = cs.vertcat(F1, x_t[2], x_t[3], u[0], u[1], c_e)
        F1 = cs.vertcat(F1, x_t[2], x_t[3], u[0], u[1], bound_dist1, bound_dist2)

    # Constraints
    # -------------------------------------
    # C = og.constraints.Rectangle([x_min, y_min, phi_min, v_x_min, v_y_min, omega_min, d_min, delta_min, -track_width],
    #                              [x_max, y_max, phi_max, v_x_max, v_y_max, omega_max, d_max, delta_max, track_width])
    # C = og.constraints.Rectangle([phi_min, v_x_min, d_min, delta_min, 0],
    #                              [phi_max, v_x_max, d_max, delta_max, track_width])
    C = og.constraints.Rectangle([phi_min, v_x_min, d_min, delta_min, 0, -track_width],
                                 [phi_max, v_x_max, d_max, delta_max, track_width, 0])

    # Code Generation
    # -------------------------------------
    problem = og.builder.Problem(u_seq, x0, cost) \
        .with_aug_lagrangian_constraints(F1, C)

    build_config = og.config.BuildConfiguration() \
        .with_build_directory("mpcc_python_build_1") \
        .with_tcp_interface_config()

    meta = og.config.OptimizerMeta().with_optimizer_name("mpcc_optimizer") \
        .with_authors("Darina Abaffyova")

    solver_config = og.config.SolverConfiguration() \
        .with_initial_penalty(256) \
        .with_max_duration_micros(50000)  # 0.05s

    builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                              build_config, solver_config)
    builder.build()
