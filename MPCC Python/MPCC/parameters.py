# Author: Darina Abaffyová
# Created: 16/05/2020

# Vehicle parameters TODO - get all these from FEB
# -------------------------------------
I_z = 323  # [kg m^2] moment of inertia of the vehicle  TODO - ASK FOR THIS VALUE!!!!!!!
m = 208  # [kg] mass of the vehicle
l_f = 0.845  # [m] length of the front part of the vehicle (this is from front axle to COG)
l_r = 0.690  # [m] length of the rear part of the vehicle  (this is from front axle to COG)

weight_f = l_r / (l_f + l_r)
weight_r = l_f / (l_f + l_r)

C_m1 = 0.287
C_m2 = 0.0565
C_r0 = 0.0518
C_r2 = 0.00035

B_r = 3.3852
C_r = 1.2691
D_r = 0.1737

B_f = 2.579
C_f = 1.2
D_f = 0.192

# Friction ellipse - TODO
p_long = 0.9
p_ellipse = 0.95

# TODO Model limits (TBD) - THESE STILL NEED TO BE CHANGED TO THE ONES CORRESPONDING TO THE FEB FORMULA
x_max = 100
x_min = -x_max
y_max = 100
y_min = -y_max
phi_max = 10
phi_min = -phi_max
v_x_max = 3.5
v_x_min = -v_x_max
v_y_max = 5
v_y_min = -v_y_max
omega_max = 70
omega_min = -omega_max
# Control limits
# Acceleration
a_max = 2
a_min = -0.1
# Steering angle
delta_max = 0.506  # 0.506 rad =  29 degrees
delta_min = -delta_max

# Track parameters
# -------------------------------------
track_width = 3

# Optimizer parameters
# -------------------------------------
N = 40  # Prediction Horizon (in time steps)
nu = 2  # Number of Decision Variables (input)
nx = 4  # Number of Parameters (state)
Ts = 0.05  # Sampling time (length of one time step in seconds)

# Weights in cost function
track_error_weight = 0.1
in_weight = [1e-4, 1e-4]  # acceleration, steering angle
in_change_weight = [0.01, 0.1]  # change of acceleration, change of steering angle
