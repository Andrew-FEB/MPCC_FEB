/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Simulator.h
 * Author: Darina Abaffyova
 *
 * Created on April 6, 2020, 12:58 PM
 */

#ifndef SIMULATOR_H
#define SIMULATOR_H

class Simulator {
public:
    Simulator();
    
private:
    int simulationSteps = 100;
    bool kinematic = true;
    double predictionHorizon = 2;
    
};

#endif /* SIMULATOR_H */