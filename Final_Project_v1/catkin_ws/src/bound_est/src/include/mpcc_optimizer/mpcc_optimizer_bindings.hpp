/* This is an auto-generated file made from optimization engine: https://crates.io/crates/optimization_engine */

#pragma once



#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <new>

static const uintptr_t MPCC_OPTIMIZER_N1 = 80;

static const uintptr_t MPCC_OPTIMIZER_N2 = 0;

static const uintptr_t MPCC_OPTIMIZER_NUM_DECISION_VARIABLES = 80;

static const uintptr_t MPCC_OPTIMIZER_NUM_PARAMETERS = 88;

/// mpcc_optimizer version of ExitStatus
/// Structure: `mpcc_optimizerExitStatus`
enum class mpcc_optimizerExitStatus {
  /// The algorithm has converged
  /// All termination criteria are satisfied and the algorithm
  /// converged within the available time and number of iterations
  mpcc_optimizerConverged,
  /// Failed to converge because the maximum number of iterations was reached
  mpcc_optimizerNotConvergedIterations,
  /// Failed to converge because the maximum execution time was reached
  mpcc_optimizerNotConvergedOutOfTime,
  /// If the gradient or cost function cannot be evaluated internally
  mpcc_optimizerNotConvergedCost,
  /// Computation failed and NaN/Infinite value was obtained
  mpcc_optimizerNotConvergedNotFiniteComputation,
};

/// Solver cache (structure `mpcc_optimizerCache`)
struct mpcc_optimizerCache;

/// mpcc_optimizer version of AlmOptimizerStatus
/// Structure: `mpcc_optimizerSolverStatus`
struct mpcc_optimizerSolverStatus {
  /// Exit status
  mpcc_optimizerExitStatus exit_status;
  /// Number of outer iterations
  unsigned long num_outer_iterations;
  /// Total number of inner iterations
  /// This is the sum of the numbers of iterations of
  /// inner solvers
  unsigned long num_inner_iterations;
  /// Norm of the fixed-point residual of the the problem
  double last_problem_norm_fpr;
  /// Total solve time
  unsigned long long solve_time_ns;
  /// Penalty value
  double penalty;
  /// Norm of delta y divided by the penalty parameter
  double delta_y_norm_over_c;
  /// Norm of F2(u)
  double f2_norm;
  /// Value of cost function at solution
  double cost;
  /// Lagrange multipliers
  double lagrange[MPCC_OPTIMIZER_N1];
};

extern "C" {

/// Deallocate the solver's memory, which has been previously allocated
/// using `mpcc_optimizer_new`
void mpcc_optimizer_free(mpcc_optimizerCache *instance);

/// Allocate memory and setup the solver
mpcc_optimizerCache *mpcc_optimizer_new();

/// Solve the parametric optimization problem for a given parameter
/// .
/// .
/// Arguments:
/// - `instance`: re-useable instance of AlmCache, which should be created using
/// `mpcc_optimizer_new` (and should be destroyed once it is not
/// needed using `mpcc_optimizer_free`
/// - `u`: (on entry) initial guess of solution, (on exit) solution
/// (length: `MPCC_OPTIMIZER_NUM_DECISION_VARIABLES`)
/// - `params`:  static parameters of the optimizer
/// (length: `MPCC_OPTIMIZER_NUM_PARAMETERS`)
/// - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
/// be used; length: `MPCC_OPTIMIZER_N1`)
/// - `c0`: Initial penalty parameter (provide `0` to use the default initial
/// penalty parameter
/// .
/// .
/// Returns:
/// Instance of `mpcc_optimizerSolverStatus`, with the solver status
/// (e.g., number of inner/outer iterations, measures of accuracy, solver time,
/// and the array of Lagrange multipliers at the solution).
mpcc_optimizerSolverStatus mpcc_optimizer_solve(mpcc_optimizerCache *instance,
                                                double *u,
                                                const double *params,
                                                const double *y0,
                                                const double *c0);

} // extern "C"
