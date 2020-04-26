/* 
 * File:   MPCController.cpp
 * Author: Darina Abaffyova
 * 
 * Created on April 6, 2020, 12:58 PM
 */

#include "mpccontroller.h"

MPCController::MPCController() {}
MPCController::MPCController(shared_ptr<Visualisation> vis) : visualisation(vis) {}
MPCController::MPCController(int ph, double dt) : predictionHorizon(ph), timeStep(dt) {}
MPCController::MPCController(int ph, double dt, shared_ptr<Visualisation> vis) : predictionHorizon(ph), timeStep(dt), visualisation(vis) {}

ControlInputs MPCController::solve(const Car &current, Track &t) const
{
    ControlInputs ci;
    auto pos = current.getPosition();
    auto vel = current.getVelocity();

    // Obtain reference and track constraints
    auto dist = calculateDistance(vel);
    auto ref = t.getCentreLine(dist);

    /* parameters */
    double p[MPCC_OPTIMIZER_NUM_PARAMETERS] = {pos.p.x, pos.p.y, vel.omega, vel.vx,                  // Current state
                                               ref.goal.x, ref.goal.y, vel.omega, 2.5,               // Reference position
                                               ref.slope, ref.nearest_point.x, ref.nearest_point.y}; // Nearest point on centreline

    /* initial guess */
    double u[MPCC_OPTIMIZER_NUM_DECISION_VARIABLES] = {0};

    /* initial penalty */
    double initPenalty = 7.0;

    /* initial lagrange mult. */
    double y[MPCC_OPTIMIZER_N1] = {0.0};

    cout << "PARAMETERS:" << endl;
    for (int i = 0; i < MPCC_OPTIMIZER_NUM_PARAMETERS; ++i)
    {
        printf("p[%d] = %g\n", i, p[i]);
    }
    /* obtain cache */
    mpcc_optimizerCache *cache = mpcc_optimizer_new();

    /* solve */
    mpcc_optimizerSolverStatus status = mpcc_optimizer_solve(cache, u, p, y, &initPenalty);

    // Deal with failed solution
    switch (status.exit_status)
    {
    case mpcc_optimizerExitStatus::mpcc_optimizerNotConvergedNotFiniteComputation:
        ci = {0, 0};
        cout << "Control Inputs NOT CONVERGED (D, delta) = (" << ci.D << ", " << ci.delta << ")" << endl;
        break;

    default:
        ci = {u[0], u[1]};
        cout << "Control Inputs (D, delta) = (" << ci.D << ", " << ci.delta << ")" << endl;
        break;
    }

    printf("\n-------------------------------------------------\n");
    printf("  Solver Statistics\n");
    printf("-------------------------------------------------\n");
    printf("exit status      : %d\n", status.exit_status);
    printf("iterations       : %lu\n", status.num_inner_iterations);
    printf("outer iterations : %lu\n", status.num_outer_iterations);
    printf("solve time       : %f ms\n", (double)status.solve_time_ns / 1000000.0);
    printf("penalty          : %f\n", status.penalty);

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
    vector<pair<coord, coord>> path;
    coord prev = car.getPosition().p;
    // Every odd value in the inputs array is ControlInputs.D,
    // while every even value is ControlInputs.delta
    for (int i = 0; i < MPCC_OPTIMIZER_NUM_DECISION_VARIABLES; i += 2)
    {
        // Calculate vector of coordinates
        c.updateCar({inputs[i], inputs[i+1]}, timeStep);
        path.push_back(make_pair(prev, c.getPosition().p));
        prev = c.getPosition().p;
    }

    visualisation->showNodeParentLinks(path);

}

double MPCController::calculateDistance(Vel &velocity) const
{
    auto tanVel = velocity.vx; //atan2(velocity.vy, velocity.vx);
    return predictionHorizon * timeStep * tanVel;
}