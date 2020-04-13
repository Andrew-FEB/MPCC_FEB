/* 
 * File:   Simulator.h
 * Author: Darina Abaffyova
 *
 * Created on April 6, 2020, 12:58 PM
 */

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>
#include <tuple>
#include <string>
#include <memory>
#include "mpcc_optimizer_bindings.hpp"

#include "car.h"
#include "track.h"
#include "definitions.h"

using namespace std;

class MPCController {
public:
    MPCController(int simSteps, double ph);
    ControlInputs solve(Car & current, Track & t);
    void discretizeModelRungeKutta(Car & car, ControlInputs control, TireForces forces,
                                    double dt, Car (*vehicleModel)(Car, ControlInputs, TireForces));
    
private:
    TrackContraints getTrackBoundary(Car & current, Track & track) const;
    double calculateDistance();
    // void initSolver();
    // void freeSolver();
    
private:
    int simulationSteps = 100;
    double predictionHorizon = 2; // in [s]
    
};

#endif /* SIMULATOR_H */
