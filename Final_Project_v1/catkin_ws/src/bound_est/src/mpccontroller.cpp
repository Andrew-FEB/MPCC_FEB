/* 
 * File:   MPCController.cpp
 * Author: Darina Abaffyova
 * 
 * Created on April 6, 2020, 12:58 PM
 */

#include "mpccontroller.h"

MPCController::MPCController() {}
MPCController::MPCController(std::shared_ptr<Visualisation> vis) : visualisation(vis) {}
MPCController::MPCController(int ph, double dt) : predictionHorizon(ph), timeStep(dt) {}
MPCController::MPCController(int ph, double dt, std::shared_ptr<Visualisation> vis) : predictionHorizon(ph), timeStep(dt), visualisation(vis) {}

ControlInputs MPCController::solve(const Car &current, Track &t) const
{
    ControlInputs ci;
    auto pos = current.getPosition();
    auto vel = current.getVelocity();

    // Obtain reference and track constraints
    auto dist = calculateDistance(vel);
    auto ref = t.getReferencePath(dist, predictionHorizon);
    
    // Arrange the reference path such that it fits the solver requirements (just c arrays)
    double slopes[predictionHorizon * 2];
    double intercepts[predictionHorizon * 2];
    double track_widths[predictionHorizon];
    double reference_points[predictionHorizon * 2];
    int i = 0;
    for (MPC_targets t : ref) {
        slopes[i] = t.left_boundary.phi;
        slopes[i+1] = t.right_boundary.phi;
        intercepts[i] = t.left_boundary.p.y - t.left_boundary.phi * t.left_boundary.p.x;
        intercepts[i+1] = t.right_boundary.p.y - t.right_boundary.phi * t.right_boundary.p.x;
        track_widths[i] = sqrt(distBetweenPoints(t.left_boundary.p, t.right_boundary.p));
        reference_points[i] = t.reference_point.x;
        reference_points[i+1] = t.reference_point.y;
        i += 2;
    }

    /* parameters */
    double p[MPCC_OPTIMIZER_NUM_PARAMETERS] = {pos.p.x, pos.p.y, vel.omega, vel.vx,       // Current state (=4)
                                               *slopes, *intercepts, *track_widths,       // Boundaries (Prediction Horizon * (Slopes(=2) + Intercepts(=2) + Width(=1))
                                               *reference_points};                        // Reference Line (Prediction Horizon * Reference Point(=2))
    /* initial guess */
    double u[MPCC_OPTIMIZER_NUM_DECISION_VARIABLES] = {0};

    /* initial penalty */
    double initPenalty = 10.0;

    /* initial lagrange mult. */
    double y[MPCC_OPTIMIZER_N1] = {0.0};

    #ifdef DEBUG
        std::unique_ptr<BoundaryLogger> log = std::make_unique<BoundaryLogger>("MPCC_CONTROLLER", "solve()", reset_logs);
        std::stringstream ss;
        log->write(ss << "PARAMETERS:");
        for (int i = 0; i < MPCC_OPTIMIZER_NUM_PARAMETERS; ++i)
        {
            log->write(ss << "p[" << i << "] = " << p[i]);
        }
    #endif

    /* obtain cache */
    mpcc_optimizerCache *cache = mpcc_optimizer_new();

    /* solve */
    mpcc_optimizerSolverStatus status = mpcc_optimizer_solve(cache, u, p, y, &initPenalty);

    // Deal with failed solution
    switch (status.exit_status)
    {
    case mpcc_optimizerExitStatus::mpcc_optimizerNotConvergedNotFiniteComputation:
        ci = {0, 0};
        #ifdef DEBUG
            log->write(ss << "Control Inputs NOT CONVERGED (D, delta) = (" << ci.D << ", " << ci.delta << ")");
        #endif
        break;

    default:
        ci = {u[0], u[1]};
        #ifdef DEBUG
            log->write(ss << "Control Inputs (D, delta) = (" << ci.D << ", " << ci.delta << ")");
        #endif
        break;
    }

    #ifdef DEBUG
        int exit_status = -1;
        switch (status.exit_status)
        {
        case mpcc_optimizerExitStatus::mpcc_optimizerConverged:
            exit_status = 0;
            break;

        case mpcc_optimizerExitStatus::mpcc_optimizerNotConvergedIterations:
            exit_status = 1;
            break;
        
        case mpcc_optimizerExitStatus::mpcc_optimizerNotConvergedOutOfTime:
            exit_status = 2;
            break;

        case mpcc_optimizerExitStatus::mpcc_optimizerNotConvergedCost:
            exit_status = 3;
            break;

        case mpcc_optimizerExitStatus::mpcc_optimizerNotConvergedNotFiniteComputation:
            exit_status = 4;
            break;
        }


        log->write(ss << "-------------------------------------------------");
        log->write(ss << "  Solver Statistics");
        log->write(ss << "-------------------------------------------------");
        log->write(ss << "exit status      : " << exit_status);
        log->write(ss << "iterations       : " << status.num_inner_iterations);
        log->write(ss << "outer iterations : " << status.num_outer_iterations);
        log->write(ss << "solve time       : " << (double)status.solve_time_ns / 1000000.0 << " ms");
        log->write(ss << "penalty          : " << status.penalty);
    #endif

    #ifdef VISUALISE
        showPredictedPath(current, u);
    #endif

    /* free memory */
    mpcc_optimizer_free(cache);

    return ci;
}

void MPCController::showPredictedPath(const Car & car, double * inputs) const
{
    Car c(car.getPosition(), car.getVelocity());
    std::vector<std::pair<coord, coord>> path;
    coord prev = car.getPosition().p;
    // Every odd value in the inputs array is ControlInputs.D,
    // while every even value is ControlInputs.delta
    for (int i = 0; i < MPCC_OPTIMIZER_NUM_DECISION_VARIABLES; i += 2)
    {
        // Calculate vector of coordinates
        c.updateCar({inputs[i], inputs[i+1]}, timeStep);
        path.push_back(std::make_pair(prev, c.getPosition().p));
        prev = c.getPosition().p;
    }

    visualisation->showNodeParentLinks(path);

}

double MPCController::calculateDistance(Vel &velocity) const
{
    auto vel = velocity.vx;
    return predictionHorizon * timeStep * vel;
}