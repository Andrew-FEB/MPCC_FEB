/* 
 * File:   Simulator.h
 * Author: Darina Abaffyova
 *
 * Created on April 6, 2020, 12:58 PM
 */

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>
#include "mpcc_optimizer_bindings.hpp"

using namespace std;
        
struct State {
    int x;
    int y;
    int phi;
    int v_x;
    int v_y;
    int omega;
};

class Simulator {
public:
    Simulator();
    Simulator(int simSteps, bool kin, double ph);
    void runSimulation(vector<State> track);
    
private:
    int simulationSteps = 100;
    bool kinematic = true;
    double predictionHorizon = 2;
    
};

#endif /* SIMULATOR_H */
