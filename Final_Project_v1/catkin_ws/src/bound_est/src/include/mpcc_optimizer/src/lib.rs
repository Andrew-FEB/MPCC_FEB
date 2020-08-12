//
// Auto-generated file by OptimizationEngine
// See https://alphaville.github.io/optimization-engine/
//
// Generated at: 2020-08-10 22:01:51.767691
//

use icasadi_mpcc_optimizer;
use libc::{c_double, c_ulong, c_ulonglong};

use optimization_engine::{constraints::*, panoc::*, alm::*, *};

// ---Private Constants----------------------------------------------------------------------------------

/// Tolerance of inner solver
const EPSILON_TOLERANCE: f64 = 0.01;

/// Initial tolerance
const INITIAL_EPSILON_TOLERANCE: f64 = 0.01;

/// Update factor for inner tolerance
const EPSILON_TOLERANCE_UPDATE_FACTOR: f64 = 0.1;

/// Delta tolerance
const DELTA_TOLERANCE: f64 = 0.01;

/// LBFGS memory
const LBFGS_MEMORY: usize = 10;

/// Maximum number of iterations of the inner solver
const MAX_INNER_ITERATIONS: usize = 500;

/// Maximum number of outer iterations
const MAX_OUTER_ITERATIONS: usize = 10;

/// Maximum execution duration in microseconds
const MAX_DURATION_MICROS: u64 = 100000;

/// Penalty update factor
const PENALTY_UPDATE_FACTOR: f64 = 4.0;

/// Initial penalty
const INITIAL_PENALTY_PARAMETER: f64 = 15.0;

/// Sufficient decrease coefficient
const SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT: f64 = 0.1;


// ---Public Constants-----------------------------------------------------------------------------------

/// Number of decision variables
pub const MPCC_OPTIMIZER_NUM_DECISION_VARIABLES: usize = 80;

/// Number of parameters
pub const MPCC_OPTIMIZER_NUM_PARAMETERS: usize = 88;

/// Number of parameters associated with augmented Lagrangian
pub const MPCC_OPTIMIZER_N1: usize = 80;

/// Number of penalty constraints
pub const MPCC_OPTIMIZER_N2: usize = 0;

// ---Export functionality from Rust to C/C++------------------------------------------------------------

/// Solver cache (structure `mpcc_optimizerCache`)
///
#[allow(non_camel_case_types)]
#[no_mangle]
pub struct mpcc_optimizerCache {
    cache: AlmCache,
}

impl mpcc_optimizerCache {
    pub fn new(cache: AlmCache) -> Self {
        mpcc_optimizerCache { cache }
    }
}

/// mpcc_optimizer version of ExitStatus
/// Structure: `mpcc_optimizerExitStatus`
#[allow(non_camel_case_types)]
#[repr(C)]
#[no_mangle]
pub enum mpcc_optimizerExitStatus {
    /// The algorithm has converged
    ///
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
}

/// mpcc_optimizer version of AlmOptimizerStatus
/// Structure: `mpcc_optimizerSolverStatus`
///
#[repr(C)]
#[no_mangle]
pub struct mpcc_optimizerSolverStatus {
    /// Exit status
    exit_status: mpcc_optimizerExitStatus,
    /// Number of outer iterations
    num_outer_iterations: c_ulong,
    /// Total number of inner iterations
    ///
    /// This is the sum of the numbers of iterations of
    /// inner solvers
    num_inner_iterations: c_ulong,
    /// Norm of the fixed-point residual of the the problem
    last_problem_norm_fpr: c_double,
    /// Total solve time
    solve_time_ns: c_ulonglong,
    /// Penalty value
    penalty: c_double,
    /// Norm of delta y divided by the penalty parameter
    delta_y_norm_over_c: c_double,
    /// Norm of F2(u)
    f2_norm: c_double,
    /// Value of cost function at solution
    cost: c_double,
    /// Lagrange multipliers
    lagrange: [c_double; MPCC_OPTIMIZER_N1]
    }

/// Allocate memory and setup the solver
#[no_mangle]
pub extern "C" fn mpcc_optimizer_new() -> *mut mpcc_optimizerCache {
    Box::into_raw(Box::new(mpcc_optimizerCache::new(initialize_solver())))
}

/// Solve the parametric optimization problem for a given parameter
/// .
/// .
/// Arguments:
/// - `instance`: re-useable instance of AlmCache, which should be created using
///   `mpcc_optimizer_new` (and should be destroyed once it is not
///   needed using `mpcc_optimizer_free`
/// - `u`: (on entry) initial guess of solution, (on exit) solution
///   (length: `MPCC_OPTIMIZER_NUM_DECISION_VARIABLES`)
/// - `params`:  static parameters of the optimizer
///   (length: `MPCC_OPTIMIZER_NUM_PARAMETERS`)
/// - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
///   be used; length: `MPCC_OPTIMIZER_N1`)
/// - `c0`: Initial penalty parameter (provide `0` to use the default initial
///   penalty parameter
/// .
/// .
/// Returns:
/// Instance of `mpcc_optimizerSolverStatus`, with the solver status
/// (e.g., number of inner/outer iterations, measures of accuracy, solver time,
/// and the array of Lagrange multipliers at the solution).
///
#[no_mangle]
pub extern "C" fn mpcc_optimizer_solve(
    instance: *mut mpcc_optimizerCache,
    u: *mut c_double,
    params: *const c_double,
    y0: *const c_double,
    c0: *const c_double,
) -> mpcc_optimizerSolverStatus {

    // Convert all pointers into the required data structures
    let instance: &mut mpcc_optimizerCache = unsafe {
        assert!(!instance.is_null());
        &mut *instance
    };

    // "*mut c_double" to "&mut [f64]"
    let u : &mut [f64] = unsafe {
        assert!(!u.is_null());
        std::slice::from_raw_parts_mut(u as *mut f64, MPCC_OPTIMIZER_NUM_DECISION_VARIABLES)
    };

    // "*const c_double" to "&[f64]"
    let params : &[f64] = unsafe {
        assert!(!params.is_null());
        std::slice::from_raw_parts(params as *const f64, MPCC_OPTIMIZER_NUM_PARAMETERS)
    };

    let c0_option: Option<f64> = if c0.is_null() {
        None::<f64>
    } else {
        Some(unsafe { *c0 })
    };

    let y0_option: Option<Vec<f64>> = if y0.is_null() {
        None::<Vec<f64>>
    } else {
        Some(unsafe { std::slice::from_raw_parts(y0 as *mut f64, MPCC_OPTIMIZER_N1).to_vec() })
    };

    // Invoke `solve`
    let status = solve(params,&mut instance.cache, u, &y0_option, &c0_option);

    // Check solution status and cast it as `mpcc_optimizerSolverStatus`
    match status {
        Ok(status) => mpcc_optimizerSolverStatus {
            exit_status: match status.exit_status() {
                core::ExitStatus::Converged => mpcc_optimizerExitStatus::mpcc_optimizerConverged,
                core::ExitStatus::NotConvergedIterations => mpcc_optimizerExitStatus::mpcc_optimizerNotConvergedIterations,
                core::ExitStatus::NotConvergedOutOfTime => mpcc_optimizerExitStatus::mpcc_optimizerNotConvergedOutOfTime,
            },
            num_outer_iterations: status.num_outer_iterations() as c_ulong,
            num_inner_iterations: status.num_inner_iterations() as c_ulong,
            last_problem_norm_fpr: status.last_problem_norm_fpr(),
            solve_time_ns: status.solve_time().as_nanos() as c_ulonglong,
            penalty: status.penalty() as c_double,
            delta_y_norm_over_c: status.delta_y_norm_over_c() as c_double,
            f2_norm: status.f2_norm() as c_double,
            cost: status.cost() as c_double,
            lagrange: match status.lagrange_multipliers() {
                Some(y) => {
                    let mut y_array : [f64; MPCC_OPTIMIZER_N1] = [0.0; MPCC_OPTIMIZER_N1];
                    y_array.copy_from_slice(&y);
                    y_array
                
                },
                None => {
                    [0.0; MPCC_OPTIMIZER_N1]
                }
            }
        },
        Err(e) => mpcc_optimizerSolverStatus {
            exit_status: match e {
                SolverError::Cost => mpcc_optimizerExitStatus::mpcc_optimizerNotConvergedCost,
                SolverError::NotFiniteComputation => mpcc_optimizerExitStatus::mpcc_optimizerNotConvergedNotFiniteComputation,
            },
            num_outer_iterations: std::u64::MAX as c_ulong,
            num_inner_iterations: std::u64::MAX as c_ulong,
            last_problem_norm_fpr: std::f64::INFINITY,
            solve_time_ns: std::u64::MAX as c_ulonglong,
            penalty: std::f64::INFINITY as c_double,
            delta_y_norm_over_c: std::f64::INFINITY as c_double,
            f2_norm: std::f64::INFINITY as c_double,
            cost: std::f64::INFINITY as c_double,
            lagrange:[0.0; MPCC_OPTIMIZER_N1]
        },
    }
}

/// Deallocate the solver's memory, which has been previously allocated
/// using `mpcc_optimizer_new`
#[no_mangle]
pub extern "C" fn mpcc_optimizer_free(instance: *mut mpcc_optimizerCache) {
    // Add impl
    unsafe {
        assert!(!instance.is_null());
        Box::from_raw(instance);
    }
}


// ---Parameters of the constraints----------------------------------------------------------------------

const CONSTRAINTS_XMIN :Option<&[f64]> = Some(&[-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,-0.1,-0.506,]);
const CONSTRAINTS_XMAX :Option<&[f64]> = Some(&[2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,2.0,0.506,]);





// ---Parameters of ALM-type constraints (Set C)---------------------------------------------------------
const SET_C_XMIN :Option<&[f64]> = Some(&[-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,-3.5,std::f64::NEG_INFINITY,]);
const SET_C_XMAX :Option<&[f64]> = Some(&[3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,3.5,0.0,]);




// ---Parameters of ALM-type constraints (Set Y)---------------------------------------------------------
/// Y_min
const SET_Y_XMIN :Option<&[f64]> = Some(&[-1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0, -1000000000000.0, 0.0]);

/// Y_max
const SET_Y_XMAX :Option<&[f64]> = Some(&[1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0, 1000000000000.0]);




// ---Internal private helper functions------------------------------------------------------------------

/// Make constraints U
fn make_constraints() -> impl Constraint {

    let bounds = Rectangle::new(CONSTRAINTS_XMIN, CONSTRAINTS_XMAX);
    bounds
}

/// Make set C
fn make_set_c() -> impl Constraint {
    let set_c = Rectangle::new(SET_C_XMIN, SET_C_XMAX);
    set_c
}


/// Make set Y
fn make_set_y() -> impl Constraint {
    let set_y = Rectangle::new(SET_Y_XMIN, SET_Y_XMAX);
    set_y
}


// ---Main public API functions--------------------------------------------------------------------------


/// Initialisation of the solver
pub fn initialize_solver() -> AlmCache {
    let panoc_cache = PANOCCache::new(MPCC_OPTIMIZER_NUM_DECISION_VARIABLES, EPSILON_TOLERANCE, LBFGS_MEMORY);
    let alm_cache = AlmCache::new(panoc_cache, MPCC_OPTIMIZER_N1, MPCC_OPTIMIZER_N2);

    alm_cache
}


/// Solver interface
pub fn solve(
    p: &[f64],
    alm_cache: &mut AlmCache,
    u: &mut [f64],
    y0: &Option<Vec<f64>>,
    c0: &Option<f64>,
) -> Result<AlmOptimizerStatus, SolverError> {

    assert_eq!(p.len(), MPCC_OPTIMIZER_NUM_PARAMETERS, "Wrong number of parameters (p)");
    assert_eq!(u.len(), MPCC_OPTIMIZER_NUM_DECISION_VARIABLES, "Wrong number of decision variables (u)");

    let psi = |u: &[f64], xi: &[f64], cost: &mut f64| -> Result<(), SolverError> {
        icasadi_mpcc_optimizer::cost(&u, &xi, &p, cost);
        Ok(())
    };
    let grad_psi = |u: &[f64], xi: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        icasadi_mpcc_optimizer::grad(&u, &xi, &p, grad);
        Ok(())
    };
    
    let f1 = |u: &[f64], res: &mut [f64]| -> Result<(), SolverError> {
        icasadi_mpcc_optimizer::mapping_f1(&u, &p, res);
        Ok(())
    };
    let bounds = make_constraints();

    let set_y = make_set_y();
    let set_c = make_set_c();
    let alm_problem = AlmProblem::new(
        bounds,
        Some(set_c),
        Some(set_y),
        psi,
        grad_psi,
        Some(f1),
        NO_MAPPING,
        MPCC_OPTIMIZER_N1,
        MPCC_OPTIMIZER_N2,
    );

    let mut alm_optimizer = AlmOptimizer::new(alm_cache, alm_problem)
        .with_delta_tolerance(DELTA_TOLERANCE)
        .with_epsilon_tolerance(EPSILON_TOLERANCE)
        .with_initial_inner_tolerance(INITIAL_EPSILON_TOLERANCE)
        .with_inner_tolerance_update_factor(EPSILON_TOLERANCE_UPDATE_FACTOR)
        .with_max_duration(std::time::Duration::from_micros(MAX_DURATION_MICROS))
        .with_max_outer_iterations(MAX_OUTER_ITERATIONS)
        .with_max_inner_iterations(MAX_INNER_ITERATIONS)
        .with_initial_penalty(c0.unwrap_or(INITIAL_PENALTY_PARAMETER))
        .with_penalty_update_factor(PENALTY_UPDATE_FACTOR)
        .with_sufficient_decrease_coefficient(SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT);

    // solve the problem using `u` an the initial condition `u` and
    // initial vector of Lagrange multipliers, if provided;
    // returns the problem status (instance of `AlmOptimizerStatus`)
    if let Some(y0_) = y0 {
        let mut alm_optimizer = alm_optimizer.with_initial_lagrange_multipliers(&y0_);
        alm_optimizer.solve(u)
    } else {
        alm_optimizer.solve(u)
    }

}