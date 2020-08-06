/* 
 * File:   MPCController.cpp
 * Author: Darina Abaffyova
 * 
 * Created on April 6, 2020, 12:58 PM
 */

#include "mpccontroller.h"

MPCController::MPCController(int ph, double dt, std::shared_ptr<Visualisation> vis, Track & t) :
        prediction_horizon(ph), time_step(dt), visualisation(vis), track(t) {
            // Populate the vector of distances
            std::vector<double> dists(prediction_horizon);
            auto d = time_step * 3.5;  // 3.5 = max velocity
            for (int i = 0; i < prediction_horizon; i++) {
                dists[i] = d + pow(time_step, 2) * 2;  // 2 = max acceleration
                d = dists[i];
            }
            distances = dists;
        }

void MPCController::solve()
{
    ControlInputs ci;
    auto car = track.getCar();
    auto pos = car->getPosition();
    auto vel = car->getVelocity();

    // Obtain reference and track constraints
    auto params = track.getReferencePath(distances);
    if (params.size()<=0) {
        std::cerr<<"Empty params list in MPCController::solve()"<<std::endl;
        return;
    }
    
    /* parameters */
    // Current state (=4) + Boundaries (Prediction Horizon * (Slopes(=2) + Intercepts(=2) + Width(=1))
    // + Reference Line (Prediction Horizon * Reference Point(=2))
    double p[MPCC_OPTIMIZER_NUM_PARAMETERS] = {pos.p.x, pos.p.y, pos.phi, vel.vx};
    // Arrange the reference path into the parameters array such that it fits the solver requirements
    auto param = params[int(prediction_horizon/2)];
    p[4] = param.left_boundary.phi; // left slope
    p[5] = param.right_boundary.phi; // right slope
    p[6] = param.left_boundary.p.y - param.left_boundary.phi * param.left_boundary.p.x; // left intercept
    p[7] = param.right_boundary.p.y - param.right_boundary.phi * param.right_boundary.p.x; // right intercept
    p[8] = sqrt(distBetweenPoints(param.left_boundary.p, param.right_boundary.p)); // track width
    int i = 0;
    for (MPC_targets param : params) { // params is prediction_horizon long
        // p[4+i] = param.left_boundary.phi; // left slopes
        // p[4+i+1] = param.right_boundary.phi; // right slopes
        // p[4+2*prediction_horizon+i] = param.left_boundary.p.y - param.left_boundary.phi * param.left_boundary.p.x; // left intercepts
        // p[4+2*prediction_horizon+i+1] = param.right_boundary.p.y - param.right_boundary.phi * param.right_boundary.p.x; // right intercepts
        // p[4+4*prediction_horizon+i/2] = sqrt(distBetweenPoints(param.left_boundary.p, param.right_boundary.p)); // track widths
        p[9+i] = param.reference_point.x; // reference coordinates x
        p[9+i+1] = param.reference_point.y; // reference coordinates y
        i += 2;
    }

    /* initial guess */
    double u[MPCC_OPTIMIZER_NUM_DECISION_VARIABLES] = {0};
    for(int i = 0; i < MPCC_OPTIMIZER_NUM_DECISION_VARIABLES; i++) {
        u[i] = initial_guess[i];
    }

    /* initial penalty */
    double init_penalty = 15.0;

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
            else if (i == 6) log->write(ss << "------------- Intercepts -------------");
            else if (i == 8) log->write(ss << "------------- Track width -------------");
            else if (i == 9) log->write(ss << "------------- Reference points -------------");
            log->write(ss << "p[" << i << "] = " << p[i]);
        }
    #endif

    /* obtain cache */
    mpcc_optimizerCache *cache = mpcc_optimizer_new();

    /* solve */
    mpcc_optimizerSolverStatus status = mpcc_optimizer_solve(cache, u, p, y, &init_penalty);

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

    // Save control inputs to be used as the initial guess at the next iteration
    for(int i = 0; i < MPCC_OPTIMIZER_NUM_DECISION_VARIABLES; i++) {
        initial_guess[i] = u[i];
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
        showPredictedPath(*car, u);
    #endif

    /* free memory */
    mpcc_optimizer_free(cache);

    /* Update car state */
    car->updateCar(ci, time_step);
    #ifdef VISUALISE
        visualisation->showCar(car->getPosition());
        visualisation->showCarDirection(car->getPosition());
    #endif
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