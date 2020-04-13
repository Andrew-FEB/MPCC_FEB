/* 
 * File:   MPCController.cpp
 * Author: Darina Abaffyova
 * 
 * Created on April 6, 2020, 12:58 PM
 */

#include "mpccontroller.h"

MPCController::MPCController(int simSteps, double ph) : simulationSteps(simSteps), predictionHorizon(ph) {}

ControlInputs MPCController::solve(Car &current, Track &t)
{
    auto pos = current.getPosition();
    auto vel = current.getVelocity();
    auto phi = current.getHeadingAngle();
    auto omega = current.getAngularVelocity();

    // Obtain reference and track constraints
    auto dist = calculateDistance();
    auto ref = t.getCentreLine(dist).at(0);
    TrackContraints tc = getTrackBoundary(current, t);

    /* parameters */
    double p[MPCC_OPTIMIZER_NUM_PARAMETERS] = {};
    // pos.x, pos.y, phi, vel.vx,               // Current state
                                            //    ref.x, ref.y,                            // Reference position
                                            //    tc.centreSlope, tc.centreX, tc.centreY,  // Nearest point on centreline
                                            //    tc.upSlope, tc.upX, tc.upY,              // Nearest point on upper boundary
                                            //    tc.lowSlope, tc.lowX, tc.lowY};          // Nearest point on lower boundary

    /* initial guess */
    double u[MPCC_OPTIMIZER_NUM_DECISION_VARIABLES] = {0};

    /* initial penalty */
    double initPenalty = 256.0;

    /* initial lagrange mult. */
    double y[MPCC_OPTIMIZER_N1] = {0.0};

    /* obtain cache */
    mpcc_optimizerCache * cache = mpcc_optimizer_new();

    /* solve */
    mpcc_optimizerSolverStatus status = mpcc_optimizer_solve(cache, u, p, y, &initPenalty);
    ControlInputs ci = {u[0], u[1]};

    /* free memory */
    mpcc_optimizer_free(cache);

    return ci;
}

double MPCController::calculateDistance()
{
    return 0;
}

TrackContraints MPCController::getTrackBoundary(Car & current, Track & track) const
{
    TrackContraints tc;
    return tc;
}

void MPCController::discretizeModelRungeKutta(Car & car, ControlInputs control, TireForces forces,
                                    double dt, Car (*vehicleModel)(Car, ControlInputs, TireForces))
{
    auto k1 = vehicleModel(car, control, forces) * dt;
    auto k2 = vehicleModel(car + k1 * 0.5 * dt, control, forces) * dt;
    auto k3 = vehicleModel(car + k2 * 0.5 * dt, control, forces) * dt;
    auto k4 = vehicleModel(car + k3 * dt, control, forces) * dt;
    car = car + (k1 + k2 * 2 + k3 * 2 + k4) * (1.0 / 6.0);
}