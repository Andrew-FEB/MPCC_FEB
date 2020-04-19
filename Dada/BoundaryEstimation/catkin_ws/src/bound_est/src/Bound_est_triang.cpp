#include <memory>
#include <iostream>
#include <chrono>
#include <fstream>
#include <string>

#include <ros/ros.h>

#include "track.h"
#include "car.h"
#include "visualisation.h"
#include "mpccontroller.h"

constexpr long rosRefreshTime_ms = 1000;
constexpr double timeStep = 0.05;

int main(int argc, char *argv[])
{

//Configure visualisation essentials
#ifdef VISUALISE
    ros::init(argc, argv, "boundary_est");
    auto visualise = std::make_shared<Visualisation>();
#else
    auto visualise = nullptr;
#endif

    //Show visualisation

    //Setup essential configuration variables
    //Configure track, car and cones
    std::unique_ptr<Track> track = std::make_unique<Track>(visualise);

    MPCController mpcc{2};
    auto car = track->getCar();

    std::cout << "Entering loop" << std::endl;

    auto lastRosRefresh = std::chrono::high_resolution_clock::now();

    //Enter refresh loop
    while (1)
    {
        //MAIN LOOP
        //Collect IMU data
        //Check critical conditions
        //If outside track constraints - error state
        //Update track
        //Process track section

        //Check metadata (laps done, goal achievement, etc.)
        //Update visualisation

        if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastRosRefresh)) > std::chrono::milliseconds(rosRefreshTime_ms))
        {
            //DADA MPCC
            auto control = mpcc.solve(*car, *track);
            //Update car outputs
            car->updateCar(control, timeStep, kinematicModel);

            #ifdef VISUALISE
            visualise->refreshRosOutput();
            #endif

            lastRosRefresh = std::chrono::high_resolution_clock::now();
        }
    }
}
