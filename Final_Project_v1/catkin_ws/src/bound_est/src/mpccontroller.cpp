/* 
 * File:   MPCController.cpp
 * Author: Darina Abaffyova
 * 
 * Created on April 6, 2020, 12:58 PM
 */

#include "mpccontroller.h"

MPCController::MPCController() {}
MPCController::MPCController(std::shared_ptr<Visualisation> vis) : visualisation(vis) {}
MPCController::MPCController(int ph, double dt) : prediction_horizon(ph), time_step(dt) {}
MPCController::MPCController(int ph, double dt, std::shared_ptr<Visualisation> vis) : prediction_horizon(ph), time_step(dt), visualisation(vis) {}

ControlInputs MPCController::solve(const Car &current, Track &t) const
{
    ControlInputs ci;
    auto pos = current.getPosition();
    auto vel = current.getVelocity();

    // Obtain reference and track constraints
    auto dist = calculateDistance(vel);
    auto ref = t.getReferencePath(dist > 0 ? dist : 0.5, prediction_horizon);

    // TODO Check if ref has the correct length
    
    /* parameters */
    // Current state (=4) + Boundaries (Prediction Horizon * (Slopes(=2) + Intercepts(=2) + Width(=1))
    // + Reference Line (Prediction Horizon * Reference Point(=2))
    double p[MPCC_OPTIMIZER_NUM_PARAMETERS] = {pos.p.x, pos.p.y, vel.omega, vel.vx};
    // Arrange the reference path into the parameters array such that it fits the solver requirements
    int i = 0;
    for (MPC_targets t : ref) { // ref is prediction_horizon long
        p[4+i] = t.left_boundary.phi; // left slopes
        p[4+i+1] = t.right_boundary.phi; // right slopes
        p[4+2*prediction_horizon+i] = t.left_boundary.p.y - t.left_boundary.phi * t.left_boundary.p.x; // left intercepts
        p[4+2*prediction_horizon+i+1] = t.right_boundary.p.y - t.right_boundary.phi * t.right_boundary.p.x; // right intercepts
        p[4+4*prediction_horizon+i/2] = sqrt(distBetweenPoints(t.left_boundary.p, t.right_boundary.p)); // track widths
        p[4+5*prediction_horizon+i] = t.reference_point.x; // reference coordinates x
        p[4+5*prediction_horizon+i+1] = t.reference_point.y; // reference coordinates y
        i += 2;
    }

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
            if (i == 0) log->write(ss << "------------- Current state -------------");
            else if (i == 4) log->write(ss << "------------- Slopes -------------");
            else if (i == 4+2*prediction_horizon) log->write(ss << "------------- Intercepts -------------");
            else if (i == 4+4*prediction_horizon) log->write(ss << "------------- Track widths -------------");
            else if (i == 4+5*prediction_horizon) log->write(ss << "------------- Reference points -------------");
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
        log->write(ss << "inner iterations : " << status.num_inner_iterations);
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
    std::vector<std::pair<Coord, Coord>> path;
    Coord prev = car.getPosition().p;
    // Every odd value in the inputs array is ControlInputs.D,
    // while every even value is ControlInputs.delta
    for (int i = 0; i < MPCC_OPTIMIZER_NUM_DECISION_VARIABLES; i += 2)
    {
        // Calculate vector of coordinates
        c.updateCar({inputs[i], inputs[i+1]}, time_step);
        path.push_back(std::make_pair(prev, c.getPosition().p));
        prev = c.getPosition().p;
    }

    visualisation->showNodeParentLinks(path);

}

double MPCController::calculateDistance(Vel &velocity) const
{
    auto vel = velocity.vx;
    return prediction_horizon * time_step * vel;
}