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
#include <fstream>
#include <iostream>
#include <string>
#include "mpcc_optimizer_bindings.hpp"

#include "car.h"
#include "track.h"

using namespace std;
        
struct State {
    double x;
    double y;
    double phi;
    double v_x;
    double v_y;
    double omega;
};

struct Boundary {
    // Each of these contains slope, x position, y position
    tuple<double, double, double> track;
    tuple<double, double, double> upBound;
    tuple<double, double, double> lowBound;
};

class MPCController {
public:
    MPCController(int simSteps, double ph, State s0);
    void runSimulation();
    vector<State> readTrackFromFile(string filename) const;
    
private:
    Boundary getTrackBoundary(State curState, Track track) const;
    
private:
    int simulationSteps = 100;
    double predictionHorizon = 2;
    State firstState;
    
};

#endif /* SIMULATOR_H */
