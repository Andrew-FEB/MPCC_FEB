/* 
 * File:   main.cpp
 * Author: Darina Abaffyova
 *
 * Created on April 6, 2020, 12:40 PM
 */

#include <cstdlib>
#include "simulator.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
    int steps = 100;
    double predHor = 2;
    State start = {0, 0, 0, 0, 0, 0};
    
    Simulator s(steps, predHor, start);
    s.runSimulation();
    
    return 0;
}

