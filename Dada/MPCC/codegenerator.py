import casadi.casadi as cs
import opengen as og
import numpy as np
import parameters as p


# Author: Darina Abaffyová
# Created: 12/02/2020
# Last updated: 03/04/2020

# Model
# -------------------------------------
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
    beta = cs.arctan2(p.l_r * cs.tan(d_f), (p.l_f + p.l_r))

    # compute next state
    x_next = v * cs.cos(psi + beta)
    y_next = v * cs.sin(psi + beta)
    psi_next = v / p.l_r * cs.sin(beta)
    v_next = a

    if calc_casadi:
        return cs.vertcat(x_next, y_next, psi_next, v_next)
    else:
        return x_next, y_next, psi_next, v_next


# Runge-Kutta 4th order method
def kinetic_model_rk(state, control, dt, calc_casadi):
    if calc_casadi:
        k1 = dt * kinetic_model_temp(state, control, calc_casadi)
        k2 = dt * kinetic_model_temp(state + 0.5 * k1, control, calc_casadi)
        k3 = dt * kinetic_model_temp(state + 0.5 * k2, control, calc_casadi)
        k4 = dt * kinetic_model_temp(state + k3, control, calc_casadi)
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    else:
        k1 = dt * np.array(kinetic_model_temp(state, control, calc_casadi))
        k2 = dt * np.array(kinetic_model_temp(state + 0.5 * k1, control, calc_casadi))
        k3 = dt * np.array(kinetic_model_temp(state + 0.5 * k2, control, calc_casadi))
        k4 = dt * np.array(kinetic_model_temp(state + k3, control, calc_casadi))
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    return next_state


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
    v_x_next = 1 / p.m * (f_rx - f_fy * cs.sin(delta) + p.m * v_y * omega)
    v_y_next = 1 / p.m * (f_ry + f_fy * cs.cos(delta) - p.m * v_x * omega)
    omega_next = 1 / p.I_z * (f_fy * p.l_f * cs.cos(delta) - f_ry * p.l_r)

    if calc_casadi:
        return cs.vertcat(x_next, y_next, phi_next, v_x_next, v_y_next, omega_next)
    else:
        return x_next, y_next, phi_next, v_x_next, v_y_next, omega_next


# Runge-Kutta 4th order method
def dynamic_model_rk(state, control, calc_casadi):
    forces = pacejka_tire_forces(state, control)
    dt = p.Ts
    if calc_casadi:
        k1 = dt * dynamic_model(state, control, forces, calc_casadi)
        k2 = dt * dynamic_model(state + 0.5 * k1, control, forces, calc_casadi)
        k3 = dt * dynamic_model(state + 0.5 * k2, control, forces, calc_casadi)
        k4 = dt * dynamic_model(state + k3, control, forces, calc_casadi)
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    else:
        k1 = dt * np.array(dynamic_model(state, control, forces, calc_casadi))
        k2 = dt * np.array(dynamic_model(state + 0.5 * k1, control, forces, calc_casadi))
        k3 = dt * np.array(dynamic_model(state + 0.5 * k2, control, forces, calc_casadi))
        k4 = dt * np.array(dynamic_model(state + k3, control, forces, calc_casadi))
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    return next_state


def pacejka_tire_forces(state, control):
    # State variables
    v_x = state[3]
    v_y = state[4]
    omega = state[5]
    # Control variables
    D = control[0]
    delta = control[1]

    # Force calculations
    alpha_f = - cs.arctan2(p.l_f * omega + v_y, v_x) + delta
    alpha_r = cs.arctan2(p.l_r * omega - v_y, v_x)

    F_fy = p.D_f * cs.sin(p.C_f * cs.arctan2(p.B_f * alpha_f, 1))
    F_ry = p.D_r * cs.sin(p.C_r * cs.arctan2(p.B_r * alpha_r, 1))

    F_rx = (p.C_m1 - p.C_m2 * v_x) * D  # - C_r0 - C_r2 * v_x ** 2

    return [F_fy, F_rx, F_ry]


def normalise(val, min_val, max_val):
    return (val - min_val) / (max_val - min_val)


def cost_function(state, control, control_prev, track_error_weight, in_weight, in_change_weight):
    # TODO - add remaining costs
    cf = 0

    # TODO - NORMALISE!
    # x = normalise(state[0], p.x_min, p.x_max)
    # y = normalise(state[1], p.y_min, p.y_max)
    # v_x = normalise(state[3], p.v_x_min, p.v_x_max)
    # v_y = normalise(state[4], p.v_y_min, p.v_y_max)
    # x_ref = normalise(state[6], p.x_min, p.x_max)
    # y_ref = normalise(state[7], p.y_min, p.y_max)
    # slope = normalise(state[12], p.x_min, p.x_max)  # NOT SURE ABOUT THIS ONE TODO
    # x_nearest = normalise(state[13], p.x_min, p.x_max)
    # y_nearest = normalise(state[14], p.y_min, p.y_max)
    x = state[0]
    y = state[1]
    v_x = state[3]
    v_y = state[4]
    x_ref = state[6]
    y_ref = state[7]
    slope = state[12]
    y_inter = state[13]

    # Line: ax + by + c = 0 -> slope * x - y + y_inter = 0
    # Point: (x1, y1) -> (x, y)
    # Distance = (| a*x1 + b*y1 + c |) / (sqrt(a*a + b*b)) -> (| slope*x - y + y_inter |) / (sqrt(slope*slope + (-1)*(-1)))

    # Contouring Error = distance from reference line
    cf += track_error_weight[0] * (cs.fabs((slope * x - y + y_inter)) / (cs.sqrt(slope ** 2 + 1)))

    # Tracking Error = distance from reference point
    cf += track_error_weight[1] * cs.sqrt((x - x_ref) ** 2 + (y - y_ref) ** 2)

    # Velocity
    v = cs.sqrt(v_x ** 2 - v_y ** 2)
    cf -= track_error_weight[2] * v

    # Input Weights
    cf += in_weight[0] * control[0] ** 2
    cf += in_weight[1] * control[1] ** 2

    # Input Change Weights
    cf += in_change_weight[0] * (control[0] - control_prev[0]) ** 2
    cf += in_change_weight[1] * (control[1] - control_prev[1]) ** 2

    return cf


def cost_function_warm_start(state, control, control_prev, track_error_weight, in_weight, in_change_weight):
    # TODO - add remaining costs
    cf = 0

    x = state[0]
    y = state[1]
    v = state[3]
    x_ref = state[4]
    y_ref = state[5]
    slope = state[8]
    x_nearest = state[9]
    y_nearest = state[10]

    # y = slope * x + y_intercept
    y_inter = y_nearest - slope * x_nearest

    # Line: ax + by + c = 0 -> slope * x - y + y_inter = 0
    # Point: (x1, y1) -> (x, y)
    # Distance = (| a*x1 + b*y1 + c |) / (sqrt(a*a + b*b))

    # Contouring Error = distance from reference line
    cf += track_error_weight[0] * (cs.fabs((slope * x - y + y_inter)) / (cs.sqrt(slope ** 2 + 1)))

    # Tracking Error = distance from reference point
    cf += track_error_weight[1] * cs.sqrt((x - x_ref) ** 2 + (y - y_ref) ** 2)

    # Velocity
    cf -= track_error_weight[2] * v

    # Input Weights
    cf += in_weight[0] * control[0] ** 2
    cf += in_weight[1] * control[1] ** 2

    # Input Change Weights
    cf += in_change_weight[0] * (control[0] - control_prev[0]) ** 2
    cf += in_change_weight[1] * (control[1] - control_prev[1]) ** 2

    return cf


def generate_code(track_error_weight, in_weight, in_change_weight, lang):
    u_seq = cs.MX.sym("u", p.nu * p.N)  # Sequence of all inputs
    x0 = cs.MX.sym("x0_xref", p.nx * 2 + 2)  # Initial state (=6) + Reference point (=6) + slope (=1) + intercept (=1)

    cost = 0
    u_prev = [0, 0]
    x_t = x0[0:12]
    slope = x0[12]
    y_inter = x0[13]

    F1 = []
    for t in range(0, p.nu * p.N, p.nu):
        u = [u_seq[t], u_seq[t + 1]]
        cost += cost_function(cs.vertcat(x_t, x0[12:14]), u, u_prev, track_error_weight, in_weight,
                              in_change_weight)  # Update cost
        u_prev = u
        # Update state
        x_t = cs.vertcat(dynamic_model_rk(x_t[0:6], u, True), x_t[6:12])
        # TODO - add the missing constraints
        # Contouring Constraint
        c_e = (slope * x_t[0] - x_t[1] + y_inter) / (cs.sqrt(slope ** 2 + 1))
        # F1 = cs.vertcat(F1, x_t[0], x_t[1], x_t[2], x_t[3], x_t[4], x_t[5], c_e)
        F1 = cs.vertcat(F1, x_t[0], x_t[1], x_t[3], x_t[4], c_e)

    # Constraints
    # -------------------------------------
    # C = og.constraints.Rectangle([p.x_min, p.y_min, p.phi_min, p.v_x_min, p.v_y_min, p.omega_min, -p.track_width],
    #                               [p.x_max, p.y_max, p.phi_max, p.v_x_max, p.v_y_max, p.omega_max, p.track_width])
    C = og.constraints.Rectangle([p.x_min, p.y_min, p.v_x_min, p.v_y_min, -p.track_width],
                                 [p.x_max, p.y_max, p.v_x_max, p.v_y_max, p.track_width])
    U = og.constraints.Rectangle([p.d_min, p.delta_min], [p.d_max, p.delta_max])

    # Code Generation
    # -------------------------------------
    problem = og.builder.Problem(u_seq, x0, cost) \
        .with_constraints(U) \
        .with_aug_lagrangian_constraints(F1, C)

    if lang == 'c':
        build_config = og.config.BuildConfiguration() \
            .with_build_directory("mpcc_c_build_1") \
            .with_build_mode(og.config.BuildConfiguration.RELEASE_MODE) \
            .with_open_version('0.7.0-alpha.1') \
            .with_build_c_bindings()
    elif lang == 'p':
        build_config = og.config.BuildConfiguration() \
            .with_build_directory("mpcc_python_build_1") \
            .with_open_version('0.7.0-alpha.1') \
            .with_tcp_interface_config()

    meta = og.config.OptimizerMeta().with_optimizer_name("mpcc_optimizer") \
        .with_authors("Darina Abaffyova")

    solver_config = og.config.SolverConfiguration() \
        .with_max_outer_iterations(100) \
        .with_max_duration_micros(50000)  # 0.05s = 50000us

    builder = og.builder.OpEnOptimizerBuilder(problem,
                                              metadata=meta,
                                              build_configuration=build_config,
                                              solver_configuration=solver_config)
    builder.build()


# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
def generate_code_warm_start(track_error_weight, in_weight, in_change_weight, lang):
    u_seq = cs.MX.sym("u", p.nu * p.N)  # Sequence of all inputs
    x0 = cs.MX.sym("x0_xref", 11)  # Initial state (=4) + Reference point (=4) + slope (=1) + nearest x, y (=2)

    cost = 0
    u_prev = [0, 0]
    x_t = x0[0:8]
    slope = x0[8]
    x_nearest = x0[9]
    y_nearest = x0[10]
    y_inter = y_nearest - slope * x_nearest

    F1 = []
    for t in range(0, p.nu * p.N, p.nu):
        u = [u_seq[t], u_seq[t + 1]]
        cost += cost_function_warm_start(cs.vertcat(x_t, x0[8:11]), u, u_prev, track_error_weight, in_weight,
                                         in_change_weight)  # Update cost
        u_prev = u
        # Update state
        x_t = cs.vertcat(kinetic_model_rk(x_t[0:4], u, p.Ts, True), x_t[4:8])
        # TODO - add the missing constraints
        # Contouring Constraint
        c_e = (slope * x_t[0] - x_t[1] + y_inter) / (cs.sqrt(slope ** 2 + 1))
        F1 = cs.vertcat(F1, x_t[0], x_t[1], x_t[2], x_t[3], u[0], u[1], c_e)

    # Constraints
    # -------------------------------------
    C = og.constraints.Rectangle([p.x_min, p.y_min, p.omega_min, p.v_x_min, p.d_min, p.delta_min, -p.track_width],
                                 [p.x_max, p.y_max, p.omega_max, p.v_x_max, p.d_max, p.delta_max, p.track_width])

    # Code Generation
    # -------------------------------------
    problem = og.builder.Problem(u_seq, x0, cost) \
        .with_aug_lagrangian_constraints(F1, C)

    if lang == 'c':
        build_config = og.config.BuildConfiguration() \
            .with_build_directory("mpcc_c_build_2") \
            .with_build_mode(og.config.BuildConfiguration.RELEASE_MODE) \
            .with_open_version('0.7.0-alpha.1') \
            .with_build_c_bindings()
    elif lang == 'p':
        build_config = og.config.BuildConfiguration() \
            .with_build_directory("mpcc_python_build_2") \
            .with_open_version('0.7.0-alpha.1') \
            .with_tcp_interface_config()

    meta = og.config.OptimizerMeta().with_optimizer_name("mpcc_optimizer") \
        .with_authors("Darina Abaffyova")

    solver_config = og.config.SolverConfiguration() \
        .with_max_duration_micros(50000)  # 0.05s = 50000us

    builder = og.builder.OpEnOptimizerBuilder(problem,
                                              metadata=meta,
                                              build_configuration=build_config,
                                              solver_configuration=solver_config)
    builder.build()
