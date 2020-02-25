import casadi.casadi as cs
import opengen as og
import matplotlib.pyplot as plt
import numpy as np
import logging as lg
from scipy.optimize import minimize, Bounds, NonlinearConstraint
from ttictoc import TicToc


## Testing Cartesian Product Constraints
nu = 10
np = 1

u = cs.SX.sym('u', nu)
p = cs.SX.sym('p', np)

f = 0.5*cs.dot(u, u) + p * sum([u[i] for i in range(nu)])

set_a = og.constraints.Rectangle(None, [float('inf'), 10])
set_b = og.constraints.Rectangle([float('-inf'), -500], None)
set_c = og.constraints.Ball2([1.0, 2.0, 3.0], 1.0)
set_d = og.constraints.BallInf(None, 20.0)
idx = [1, 3, 6, 9]
set_u = og.constraints.CartesianProduct(nu, idx, [set_a, set_b, set_c, set_d])
problem = og.builder.Problem(u, p, f)
meta = og.config.OptimizerMeta()                \
    .with_version("0.0.0")                      \
    .with_authors(["P. Sopasakis", "E. Fresk"]) \
    .with_licence("CC4.0-By")                   \
    .with_optimizer_name("the_optimizer")
build_config = og.config.BuildConfiguration()  \
    .with_build_directory("python_build")      \
    .with_build_mode("debug")                  \
    .with_tcp_interface_config()
solver_config = og.config.SolverConfiguration()   \
            .with_lfbgs_memory(15)                \
            .with_tolerance(1e-5)                 \
            .with_max_inner_iterations(155)
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          metadata=meta,
                                          build_configuration=build_config,
                                          solver_configuration=solver_config)\
    .with_generate_not_build_flag(False).with_verbosity_level(3)
builder.build()

mng = og.tcp.OptimizerTcpManager('python_build/the_optimizer')
mng.start()

pong = mng.ping()                 # check if the server is alive
print(pong)
solution = mng.call([1000.0])  # call the solver over TCP

if solution.is_ok():
    solution_data = solution.get()
    u_star = solution_data.solution
    exit_status = solution_data.exit_status
    solver_time = solution_data.solve_time_ms
else:
    solver_error = solution.get()
    error_code = solver_error.code
    error_msg = solver_error.message

mng.kill()