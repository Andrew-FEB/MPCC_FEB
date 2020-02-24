import casadi.casadi as cs
import opengen as og
import numpy as np
import math

# Author: Darina Abaffyová
# Created: 12/02/2020
# Last updated: 17/02/2020

# Parameters
# -------------------------------------
# Vehicle parameters
I_z = 7  # [kg m^2] moment of inertia of the vehicle ASK FOR THIS VALUE!!!!!!!
m = 208  # [kg] mass of the vehicle
l_f = 0.845  # [m] length of the front part of the vehicle (this is from front axle to COG)
l_r = 0.690  # [m] length of the rear part of the vehicle  (this is from front axle to COG)


# Tire Parameters (TBD)
# Tire Force Curve
# B_f
# B_r
# C_f
# C_r
# D_f
# D_r

# Motor Model
# C_m1
# C_m2

# Rolling Resistance
# C_rr

# Drag
# C_d


# Model
# -------------------------------------
def vehicle_dynamics_ct():
    # ODEs of the bicycle model
    # odeX = diff(X) == v_x * cos(phi) - v_y * sin(phi);
    # odeY = diff(Y) == v_x * sin(phi) + v_y * cos(phi);
    # odePhi = diff(phi) == omega;
    # odeV_x = diff(v_x) == 1 / m * (F_rx - F_fy * sin(delta) + m * v_y * omega); % or -m * v_y * omega ???
    # odeV_y = diff(v_y) == 1 / m * (F_ry + F_fy * cos(delta) - m * v_x * omega);
    # odeOmega = diff(omega) == 1 / I_z * (l_f * F_fy * cos(delta) - l_r * F_ry);
    return 0


def vehicle_dynamics_dt(z, u, dt):
    # First simple (kinematic) model (source: https://github.com/MPC-Berkeley/barc/wiki/Car-Model):
    # get states / inputs
    x = z[0]  # Longitudinal position
    y = z[1]  # Lateral Position
    psi = z[2]  # Yaw rate
    v = z[3]  # Velocity
    d_f = u[0]  # Front steering angle
    a = u[1]  # Acceleration

    # extract parameters
    # (L_a, L_b) = vh_mdl

    # compute slip angle
    beta = math.atan(l_r / (l_f + l_r) * math.tan(d_f))

    # compute next state
    x_next = x + dt * (v * math.cos(psi + beta))
    y_next = y + dt * (v * math.sin(psi + beta))
    psi_next = psi + dt * v / l_r * math.sin(beta)
    v_next = v + dt * a

    return np.array([x_next, y_next, psi_next, v_next])


def tire_ct():
    # Force calculations
    # alpha_r = - atan((omega*l_f + v_y)/v_x);
    # alpha_f =   atan((omega*l_r - v_y)/v_x);

    # F_rx = (C_m1 - C_m2 * v_x) * d - C_rr - C_d*v_x^2;
    # F_ry = D_r * sin(C_r * atan(B_r * alpha_r));
    # F_fy = D_f * sin(C_f * atan(B_f * alpha_f));
    return 0


def tire_dt():
    return 0


# Problem
# -------------------------------------
N = 40  # Prediction Horizon (in time steps)
nu = 2  # Number of Decision Variables (input)
nx = 4  # Number of Parameters (state)
Ts = 0.05  # 0.05  # Sampling time (length of one time step)

# Not sure about the u_seq - should it be nu*N large ???
u_seq = cs.MX.sym("u", nu * N)  # Sequence of all inputs
x0 = cs.MX.sym("x0", nx)  # Initial state

x_ref = [1.0, 1.0, 0, 0]
state_error_weight = 70
input_change_weight = 30


# Cost function:
def cost_function(x, u, u_prev):
    cf = 0
    # Cost on state error
    for i in range(0, nx):
        cf += state_error_weight * (x[i] - x_ref[i]) ** 2
    # cf = 70 * (x[0] - x_ref[0]) ** 2
    # cf += 70 * (x[1] - x_ref[1]) ** 2
    # cf += 70 * (x[2] - x_ref[2]) ** 2
    # cf += 70 * (x[3] - x_ref[3]) ** 2
    # Cost on input change
    for i in range(0, nu):
        cf += input_change_weight * (u_prev[i] - u[i]) ** 2
    return cf


def vehicle_dynamics_casadi(z, u):
    # First simple (kinematic) model (source: https://github.com/MPC-Berkeley/barc/wiki/Car-Model):
    # get states, inputs
    x = z[0]
    y = z[1]
    psi = z[2]
    v = z[3]
    d_f = u[0]
    a = u[1]

    # compute slip angle
    beta = cs.atan(l_r / (l_f + l_r) * cs.tan(d_f))

    # compute next state
    x_next = v * cs.cos(psi + beta)
    y_next = v * cs.sin(psi + beta)
    psi_next = v / l_r * cs.sin(beta)
    v_next = a

    f = z + Ts * cs.vertcat(x_next, y_next, psi_next, v_next)

    return f


cost = 0
x_t = x0
for t in range(0, nu * N, nu):
    # Not sure about the u_seq - should it be t and t+1 ???
    if t > 2:
        cost += cost_function(x_t, [u_seq[t], u_seq[t + 1]], [u_seq[t - 2], u_seq[t - 1]])  # Update cost
    else:
        cost += cost_function(x_t, [u_seq[t], u_seq[t + 1]], [0, 0])  # Update cost
    x_t = vehicle_dynamics_casadi(x_t, [u_seq[t], u_seq[t + 1]])  # Update state

# Constraints
# -------------------------------------
# U = og.constraints.Rectangle([-0.75, -3], [0.75, 3])
U = og.constraints.BallInf(None, 0.95)

# Code Generation
# -------------------------------------
problem = og.builder.Problem(u_seq, x0, cost) \
    .with_constraints(U)

build_config = og.config.BuildConfiguration() \
    .with_build_directory("mpcc_python_build_0") \
    .with_tcp_interface_config()

meta = og.config.OptimizerMeta().with_optimizer_name("mpcc_optimizer") \
    .with_authors("Darina Abaffyova") \
    .with_version("0.0.0")

solver_config = og.config.SolverConfiguration() \
    .with_tolerance(1e-7) \
    .with_initial_tolerance(1e-7) \
    .with_max_outer_iterations(50000) \
    .with_delta_tolerance(1e-2) \
    .with_penalty_weight_update_factor(10.0)

builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config)
builder.build()