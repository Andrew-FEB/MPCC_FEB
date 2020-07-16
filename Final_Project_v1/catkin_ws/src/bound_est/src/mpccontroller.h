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
#include "visualisation.h"

using namespace std;

class MPCController {
public:
    MPCController();
    MPCController(shared_ptr<Visualisation> vis);
    MPCController(int ph, double dt);
    MPCController(int ph, double dt, shared_ptr<Visualisation> vis);
    ControlInputs solve(const Car & current, Track & t) const;
    
private:
    double calculateDistance(Vel & velocity) const;
    void showPredictedPath(const Car & car, double * inputs) const;
    
private:
    int predictionHorizon = 40; // in time steps
    double timeStep = 0.05; // in [s]
    shared_ptr<Visualisation> visualisation = nullptr;
    
};

#endif /* SIMULATOR_H */
