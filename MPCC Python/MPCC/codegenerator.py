import sys

import casadi.casadi as cs
import numpy as np
import opengen as og

import parameters as p


# Author: Darina Abaffyová
# Created: 12/02/2020

# Model
# -------------------------------------
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


def kinematic_model(state, control, calc_casadi):
    # get states
    x = state[0]  # Longitudinal position
    y = state[1]  # Lateral Position
    psi = state[2]  # Heading Angle
    v = state[3]  # Velocity
    # get inputs
    a = control[0]  # Acceleration
    d = control[1]  # Front steering angle

    # compute slip angle
    beta = cs.arctan2(p.l_r * cs.tan(d), (p.l_f + p.l_r))

    # compute change in state
    x_next = v * cs.cos(psi + beta)
    y_next = v * cs.sin(psi + beta)
    psi_next = v / p.l_r * cs.sin(beta)
    v_next = a

    if calc_casadi:
        return cs.vertcat(x_next, y_next, psi_next, v_next)
    else:
        return x_next, y_next, psi_next, v_next


# Runge-Kutta 4th order method
def kinematic_model_rk(state, control, calc_casadi):
    dt = p.Ts
    if calc_casadi:
        k1 = dt * kinematic_model(state, control, calc_casadi)
        k2 = dt * kinematic_model(state + 0.5 * k1, control, calc_casadi)
        k3 = dt * kinematic_model(state + 0.5 * k2, control, calc_casadi)
        k4 = dt * kinematic_model(state + k3, control, calc_casadi)
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    else:
        k1 = dt * np.array(kinematic_model(state, control, calc_casadi))
        k2 = dt * np.array(kinematic_model(state + 0.5 * k1, control, calc_casadi))
        k3 = dt * np.array(kinematic_model(state + 0.5 * k2, control, calc_casadi))
        k4 = dt * np.array(kinematic_model(state + k3, control, calc_casadi))
        next_state = state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    return next_state


# Cost Function
# -------------------------------------
def cost_function(state, ref, control, control_prev, track_weight):
    cf = 0

    x = state[0]
    y = state[1]
    v = state[3]
    x_ref = ref[0]
    y_ref = ref[1]
    # slope = tan[0]
    # intercept = tan[1]

    # Line: ax + by + c = 0 -> slope * x - y + intercept = 0
    # Point: (x1, y1) -> (x, y)
    # Distance = (| a*x1 + b*y1 + c |) / (sqrt(a*a + b*b))

    # Contouring Error = distance from reference line
    # cf += track_error_weight[0] * (cs.fabs((slope * x - y + intercept)) / (cs.sqrt(slope ** 2 + 1)))

    # Tracking Error = distance from reference point
    cf += track_weight * cs.sqrt((x - x_ref) ** 2 + (y - y_ref) ** 2)

    # Velocity
    cf -= p.velocity_weight * v

    # Input Weights
    cf += p.in_weight[0] * control[0] ** 2
    cf += p.in_weight[1] * control[1] ** 2

    # Input Change Weights
    cf += p.in_change_weight[0] * (control[0] - control_prev[0]) ** 2
    cf += p.in_change_weight[1] * (control[1] - control_prev[1]) ** 2

    return cf


# Code generation + constraints
# -------------------------------------
def generate_code(lang):
    u_seq = cs.MX.sym("u", p.nu * p.N)  # Sequence of all inputs
    # Initial state(=4) + Prediction horizon * (slopes(=2) + intercepts(=2) + widths(=1))
    # + Prediction horizon * Reference point(=2)
    x0 = cs.MX.sym("x0_xref", p.nx + p.N * 5 + p.N * 2)

    cost = 0
    u_prev = [0, 0]
    x_est = x0[0:p.nx]
    slope_list = x0[p.nx:p.nx + p.N * 2]
    intercept_list = x0[p.nx + p.N * 2:p.nx + p.N * 4]
    track_width_list = x0[p.nx + p.N * 4:p.nx + p.N * 5]
    ref_list = x0[p.nx + p.N * 5:x0.shape[0]]

    F1 = []
    track_weight = p.track_error_weight
    for t in range(0, p.nu * p.N, p.nu):
        u = [u_seq[t], u_seq[t + 1]]
        # Update cost
        cost += cost_function(x_est, ref_list[t:t + 2], u, u_prev, track_weight)
        u_prev = u
        # The weight increases in each iteration, as the last reference point
        # is more important to reach than the first one
        # if t >= 2*p.N*3/4:
        track_weight = p.track_error_weight * (1 + t / p.N)
        # Update state estimate
        x_est = kinematic_model_rk(x_est, u, True)
        # Boundary Constraint
        slope_up = slope_list[t]
        slope_low = slope_list[t + 1]
        intercept_up = intercept_list[t]
        intercept_low = intercept_list[t + 1]
        d_up = cs.fabs(slope_up * x_est[0] - x_est[1] + intercept_up) / (cs.sqrt(slope_up ** 2 + 1))
        d_low = cs.fabs(slope_low * x_est[0] - x_est[1] + intercept_low) / (cs.sqrt(slope_low ** 2 + 1))
        track_width = track_width_list[t / 2]
        F1 = cs.vertcat(F1, x_est[2], x_est[3], d_up - track_width, d_low - track_width)

    # Constraints
    # -------------------------------------
    C = og.constraints.Rectangle([p.phi_min, p.v_x_min, -np.inf, -np.inf], [p.phi_max, p.v_x_max, 0, 0])
    U = og.constraints.Rectangle([p.a_min, p.delta_min] * p.N, [p.a_max, p.delta_max] * p.N)

    # Code Generation
    # -------------------------------------
    problem = og.builder.Problem(u_seq, x0, cost) \
        .with_aug_lagrangian_constraints(F1, C) \
        .with_constraints(U)

    if lang == 'c':
        build_config = og.config.BuildConfiguration() \
            .with_build_directory("mpcc_c_build_kin") \
            .with_build_mode(og.config.BuildConfiguration.RELEASE_MODE) \
            .with_open_version('0.7.0-alpha.1') \
            .with_build_c_bindings()
    elif lang == 'p':
        build_config = og.config.BuildConfiguration() \
            .with_build_directory("mpcc_python_build_kin") \
            .with_open_version('0.7.0-alpha.1') \
            .with_tcp_interface_config()
    else:
        sys.exit("Invalid value for parameter lang - this can only be 'p' (=python) or 'c' (=C)")

    meta = og.config.OptimizerMeta().with_optimizer_name("mpcc_optimizer") \
        .with_authors("Darina Abaffyova")

    solver_config = og.config.SolverConfiguration() \
        .with_initial_tolerance(0.01) \
        .with_tolerance(0.01) \
        .with_max_outer_iterations(100) \
        .with_max_duration_micros(50000)  # 0.05s = 50000us

    builder = og.builder.OpEnOptimizerBuilder(problem,
                                              metadata=meta,
                                              build_configuration=build_config,
                                              solver_configuration=solver_config)
    builder.build()
