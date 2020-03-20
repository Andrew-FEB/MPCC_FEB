import casadi.casadi as cs
import opengen as og
import numpy as np

# Author: Darina Abaffyová
# Created: 12/02/2020
# Last updated: 20/03/2020

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
p_long = 700
p_ellipse = 500

# TODO Model limits (TBD) - THESE STILL NEED TO BE CHANGED TO THE ONES CORRESPONDING TO THE FORMULA
x_max = 26
x_min = -3.5
y_max = 17
y_min = -3
phi_max = np.pi
phi_min = -phi_max
v_x_max = 50
v_x_min = 0.5
v_y_max = 50
v_y_min = 0.5
omega_max = 2 * np.pi
omega_min = -omega_max
# Control limits
d_max = 1
d_min = -d_max
delta_max = 0.506  # [rad] =  29 degrees
delta_min = -delta_max

# Track parameters
track_width = 1.5

# Optimizer parameters
N = 40  # Prediction Horizon (in time steps)
nu = 2  # Number of Decision Variables (input)
nx = 6  # Number of Parameters (state)
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
        return [x_next, y_next, phi_next, v_x_next, v_y_next, omega_next,
                state[6], state[7], state[8], state[9], state[10], state[11]]


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
                          state[6], state[7], state[8], state[9], state[10], state[11],
                          state[12], state[13], state[14], state[15])
    else:
        return [x_next, y_next, phi_next, v_x_next, v_y_next, omega_next,
                state[6], state[7], state[8], state[9], state[10], state[11],
                state[12], state[13], state[14], state[15]]


# Runge-Kutta 4th order method
def dynamic_model_rk(state, control, forces, dt, calc_casadi):
    if calc_casadi:
        k1 = dt * dynamic_model(state, control, forces, calc_casadi)
        k2 = dt * dynamic_model(state + dt * 0.5 * k1, control, forces, calc_casadi)
        k3 = dt * dynamic_model(state + dt * 0.5 * k2, control, forces, calc_casadi)
        k4 = dt * dynamic_model(state + dt * k3, control, forces, calc_casadi)
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return cs.vertcat(next_state[0], next_state[1], next_state[2], next_state[3], next_state[4], next_state[5],
                          state[6], state[7], state[8], state[9], state[10], state[11], state[12], state[13],
                          state[14], state[15])
    else:
        k1 = dt * np.array(dynamic_model(state, control, forces, calc_casadi))
        k2 = dt * np.array(dynamic_model(state + dt * 0.5 * k1, control, forces, calc_casadi))
        k3 = dt * np.array(dynamic_model(state + dt * 0.5 * k2, control, forces, calc_casadi))
        k4 = dt * np.array(dynamic_model(state + dt * k3, control, forces, calc_casadi))
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return [next_state[0], next_state[1], next_state[2], next_state[3], next_state[4], next_state[5],
                state[6], state[7], state[8], state[9], state[10], state[11], state[12], state[13], state[14], state[15]]


def tire_forces(state, control):
    # State variables
    v_x = state[3]
    v_y = state[4]
    omega = state[5]
    # Control variables
    D = control[0]
    delta = control[1]

    # Force calculations

    alpha_f = cs.arctan2((v_y + l_f * omega), v_x) - delta
    alpha_r = cs.arctan2((v_y - l_r * omega), v_x)

    F_fy = D_f * cs.sin(C_f * cs.arctan(B_f * alpha_f))
    F_ry = D_r * cs.sin(C_r * cs.arctan(B_r * alpha_r))

    F_rx = (C_m1 - C_m2 * v_x) * D - C_rr - C_d * v_x ** 2

    # TODO Tire constraints

    return [F_fy, F_rx, F_ry]


def cost_function(state, state_prev, u, u_prev, contouring_error_weight, in_weight, in_change_weight):
    # TODO - add remaining costs
    cf = 0

    x = state[0]
    y = state[1]
    x_ref = state[6]
    y_ref = state[7]
    x_prev = state_prev[0]
    y_prev = state_prev[1]

    # y = slope * x + y_intercept
    slope = cs.arctan2(y_prev, x_prev)
    y_inter = y - slope * x

    # Line: ax + by + c = 0 -> slope * x - y + y_inter = 0
    # Point: (x1, y1) -> (x, y)
    # Distance = (| a*x1 + b*y1 + c |) / (sqrt(a*a + b*b))

    # Contouring Error
    cf += contouring_error_weight[0] * ((cs.fabs((slope * x - y + y_inter)) / (cs.sqrt(slope ** 2 + 1))) - track_width) ** 2

    # Tracking Error
    cf += contouring_error_weight[1] * cs.sqrt((x - x_ref) ** 2 + (y - y_ref) ** 2)

    # Contouring Formulation
    # e_c = cs.sin(phi) * (x - x_ref) - cs.cos(phi) * (y - y_ref)
    # e_l = - cs.cos(phi) * (x - x_ref) - cs.sin(phi) * (y - y_ref)

    # cf += contouring_error_weight[0] * e_c ** 2
    # cf += contouring_error_weight[1] * e_l ** 2
    # cf -= contouring_error_weight[2] * v ** 2

    # Cost on input
    cf += in_weight[0] * u[0] ** 2
    cf += in_weight[1] * u[1] ** 2

    # Cost on input change
    cf += in_change_weight[0] * (u_prev[0] - u[0]) ** 2
    cf += in_change_weight[1] * (u_prev[1] - u[1]) ** 2

    return cf


def generate_code(contouring_error_weight, in_weight, in_change_weight):
    # Not sure about the u_seq - should it be nu*N large ???
    u_seq = cs.MX.sym("u", nu * N)  # Sequence of all inputs
    x0 = cs.MX.sym("x0_xref", nx * 2 + 4)  # Initial state (=6) + Reference state (=6) + boundaries (=4)

    cost = 0
    x_t = x0
    x_t_prev = [0] * nx
    F1 = []
    for t in range(0, nu * N, nu):
        if t >= 2:
            u_prev = [u_seq[t - 2], u_seq[t - 1]]
        else:
            u_prev = [0, 0]

        u = [u_seq[t], u_seq[t + 1]]
        cost += cost_function(x_t, x_t_prev, u, u_prev, contouring_error_weight, in_weight, in_change_weight)  # Update cost
        f = tire_forces(x_t, u)
        x_t_prev = x_t
        x_t = dynamic_model_rk(x_t, u, f, Ts, True)  # Update state
        # TODO - add the missing constraints
        # f_fy = forces[0]
        # f_rx = forces[1]
        # f_ry = forces[2]
        F1 = cs.vertcat(F1, x_t[3], x_t[4], u[0], u[1],
                        f[2] ** 2 + (p_long * 0.5 * f[1])**2, f[0] ** 2 + (p_long * 0.5 * f[1]) ** 2) # Friction ellipse

    # Terminal Cost
    # cost += 300 * ((x_t[0] - x_t[6]) ** 2 + (x_t[1] - x_t[7]) ** 2 - track_width ** 2)

    # Constraints
    # -------------------------------------
    C = og.constraints.Rectangle([v_x_min, v_y_min, d_min, delta_min, 0, 0],
                                 [v_x_max, v_y_max, d_max, delta_max, (p_ellipse * D_r) ** 2, (p_ellipse * D_f) ** 2])

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
        .with_initial_penalty(4) \
        .with_max_outer_iterations(20) \
        .with_max_outer_iterations(100) \
        .with_max_duration_micros(500000)  # 0.5s

    builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                              build_config, solver_config)
    builder.build()
