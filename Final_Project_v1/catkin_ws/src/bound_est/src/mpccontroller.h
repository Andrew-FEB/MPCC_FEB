/* 
 * File:   MPCController.h
 * Author: Darina Abaffyova
 *
 * Created on April 6, 2020, 12:58 PM
 */

#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

#include <vector>
#include <tuple>
#include <string>
#include <memory>
#include <iostream>
#include "mpcc_optimizer_bindings.hpp"

#include "car.h"
#include "track.h"
#include "definitions.h"
#include "visualisation.h"
#include "boundaryLogger.h"
#include "boundGlobals.h"

class MPCController {
public:
    MPCController(int ph, double dt, std::shared_ptr<Visualisation> vis, Track & t);
    void solve();
    
private:
    void showPredictedPath(const Car & car, double * inputs) const;
    
private:
    int prediction_horizon = 40; // in time steps
    double time_step = 0.05; // in [s]
    double initial_guess[MPCC_OPTIMIZER_NUM_DECISION_VARIABLES] = {0};
    std::shared_ptr<Visualisation> visualisation = nullptr;
    Track & track;
    std::vector<double> distances;
};

#endif /* MPCCONTROLLER_H */
