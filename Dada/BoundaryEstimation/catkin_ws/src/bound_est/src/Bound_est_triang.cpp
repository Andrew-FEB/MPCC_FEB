#include <memory>
#include <iostream>
#include <chrono>
#include <fstream>
#include <string>

#include <ros/ros.h>

#include "track.h"
#include "car.h"
#include "visualise.h"
#include "mpccontroller.h"

constexpr long rosRefreshTime_ms = 100;

int main(int argc, char *argv[])
{
    
    //Configure visualisation essentials
    #ifdef VISUALISE
        ros::init(argc, argv, "boundary_est");
        auto visualise = std::make_shared<Visualise>();
    #else
        auto visualise = nullptr;
    #endif

    //Show visualisation

    //Setup essential configuration variables
    //Configure track, car and cones
    std::unique_ptr<Track> track = std::make_unique<Track>(visualise);
/*
	//Testing
    std::ifstream fileP("/home/dm501/catkin_ws/src/bound_est/src/resources/cone_files/cones.txt");
    if (!fileP) std::cerr<<"PANIC!!"<<std::endl;
    int count = 0;
    double x, y;
    std::string cone;
    while (std::getline(fileP, cone, ','))
    {
        if (count==0) x = std::stod(cone, 0);
        if (count==1) y = std::stod(cone, 0);
        if (count==4)
        {
            count = 0;
            track->addCone(x, y, BoundPos::undefined);
        } 
        else count++;
    }
    std::cout<<(*track);
    std::cout<<"Finished"<<std::endl;
*/
    track->addCone(-2.5, -2.0, BoundPos::left);
    track->addCone(2.5, -1.5, BoundPos::right);
    track->addCone(-2.0, 1.0, BoundPos::left);
    track->addCone(3.0, 1.5, BoundPos::right);
    track->addCone(-2.5, 4.0, BoundPos::left);
    track->addCone(2.6, 3.7, BoundPos::right);
    track->addCone(-1.0, 6.5, BoundPos::left);
    track->addCone(5.0, 7.0, BoundPos::right);
    track->addCone(-0.5, 9.0, BoundPos::left);
    track->addCone(6.7, 11.3, BoundPos::right);
    track->addCone(1.0, 13.0, BoundPos::left);
    track->addCone(8.7, 12.3, BoundPos::right);
    track->addCone(2.7, 13.6, BoundPos::left);
    track->addCone(11, 14.5, BoundPos::right);
    track->addCone(4.9, 14.2, BoundPos::left);
    track->addCone(12, 16.0, BoundPos::right);
    track->addCone(6.2, 15.5, BoundPos::left);
    track->addCone(12.5, 20.3, BoundPos::right);
    track->addCone(7.0, 19.0, BoundPos::left);

    std::cout<<"Entering loop"<<std::endl;
#ifdef VISUALISE
    auto lastRosRefresh = std::chrono::high_resolution_clock::now();
#endif
    //Enter refresh loop
    while (1)
    {
    //MAIN LOOP
    //Collect IMU data
    //Check critical conditions 
    //If outside track constraints - error state
    //Update track
    //Process track section
    //DADA MPCC
    //Update car outputs
    //Check metadata (laps done, goal achievement, etc.)
    //Update visualisation
#   ifdef VISUALISE
    if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-lastRosRefresh)) > std::chrono::milliseconds(rosRefreshTime_ms))
    {
        visualise->refreshRosOutput();
        lastRosRefresh = std::chrono::high_resolution_clock::now();
    }
    #endif
    }
   
 } 
