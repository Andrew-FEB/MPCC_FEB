/* 
 * File:   Simulator.cpp
 * Author: Darina Abaffyova
 * 
 * Created on April 6, 2020, 12:58 PM
 */

#include "simulator.h"

Simulator::Simulator(int simSteps, double ph, State s0)
        : simulationSteps(simSteps), predictionHorizon(ph), firstState(s0)
{
}

void Simulator::runSimulation() // vector<State> track
{
    auto track = readTrackFromFile("/home/dada/MPCC_FEB/Dada/MPCC/track.txt");
    State curState = firstState;
//    State refState = track[0];
//    Boundary boundaries = getTrackBoundary(curState, track);
    
    /* parameters             */
    double p[MPCC_OPTIMIZER_NUM_PARAMETERS] = {-2.4, -0.45, -0.65, 2.0,
                                               1.49, -1.85, -0.35, 1,
                                               -0.67, -1.84, -0.83,
                                               -0.67, -1.84, 0.67,
                                               -0.67, -1.84, -2.33};
//    double p[MPCC_OPTIMIZER_NUM_PARAMETERS] = {curState.x, curState.y, curState.phi, curState.v_x,
//                                               refState.x, refState.y, refState.phi, refState.v_x,
//                                               get<0>(boundaries.track), get<1>(boundaries.track), get<2>(boundaries.track),
//                                               get<0>(boundaries.upBound), get<1>(boundaries.upBound), get<2>(boundaries.upBound),
//                                               get<0>(boundaries.lowBound), get<1>(boundaries.lowBound), get<2>(boundaries.lowBound)};

    /* initial guess          */
    double u[MPCC_OPTIMIZER_NUM_DECISION_VARIABLES] = {0};

    /* initial penalty        */
    double init_penalty = 256.0;

    /* initial lagrange mult. */
    double y[MPCC_OPTIMIZER_N1] = {0.0};

    /* obtain cache           */
    mpcc_optimizerCache *cache = mpcc_optimizer_new();

    /* solve                  */
    mpcc_optimizerSolverStatus status = mpcc_optimizer_solve(cache, u, p, y, &init_penalty);
    
    /* print results */
    printf("\n\n-------------------------------------------------\n");
    printf("  Solution\n");
    printf("-------------------------------------------------\n");

    for (int i = 0; i < MPCC_OPTIMIZER_NUM_DECISION_VARIABLES; ++i) {
        printf("u[%d] = %g\n", i, u[i]);
    }

    /* free memory */
    mpcc_optimizer_free(cache);
    
}

vector<State> Simulator::readTrackFromFile(string filename) const
{
    fstream file;
    file.open(filename, ios::in);
    tuple<double, double> track;
    double up;
    double low;
    
    for (string line; getline(file, line); ) {
        
    }
    
    file.close();
}

Boundary Simulator::getTrackBoundary(State curState, vector<State> track) const
{
    return {{0,0,0}, {0,0,0}, {0,0,0}};
}