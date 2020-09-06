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
    MPCController(std::shared_ptr<Visualisation> vis, Track & t, bool simulating = false);
    /**
     * Using the generated bindings, the generated solver is called, with the necessary
     * parameters (current position of the car, reference, boundaries) and the calculated
     * controls are ouputted. If  the bool simulating is set to true, also the simulated
     * car is updated appropriately.
     */
    ControlInputs solve();
    
private:
    /**
     * The predicted path is calculated from the sequence of control inputs calculated
     * in the solve function, from which this function is called.
     */
    void showPredictedPath(const Car & car, double * inputs, bool converged) const;
    
private:
    int prediction_horizon = MPCC_OPTIMIZER_NUM_DECISION_VARIABLES/2; // in time steps
    double time_step = 0.05; // in [s]
    double initial_guess[MPCC_OPTIMIZER_NUM_DECISION_VARIABLES] = {0};
    std::shared_ptr<Visualisation> visualisation = nullptr;
    Track & track;
    std::vector<double> distances;
    bool simulating;
};

#endif /* MPCCONTROLLER_H */
