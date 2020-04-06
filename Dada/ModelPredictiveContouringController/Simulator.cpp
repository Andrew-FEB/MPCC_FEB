/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Simulator.cpp
 * Author: Darina Abaffyova
 * 
 * Created on April 6, 2020, 12:58 PM
 */

#include "Simulator.h"

Simulator::Simulator(){}

Simulator::Simulator(int simSteps, bool kin, double ph)
        : simulationSteps(simSteps), kinematic(kin), predictionHorizon(ph)
{
    
}

