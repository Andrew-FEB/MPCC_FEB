#include <memory>
#include <iostream>
#include <chrono>
#include <string>

#include <ros/ros.h>
//#include "bound_est/ConeMap.h"

#include "track.h"
#include "car.h"
#include "visualisation.h"
#include "mpccontroller.h"

#ifndef DEBUG
    #undef DDEBUG_LOOPS
    #undef DDEBUG_SLOW
    #undef VISUALISE
#endif

constexpr long ROS_REFRESH_TIME_MS = 1000;

//Configure globals
#ifdef DEBUG
    bool reset_logs = true;
    constexpr int DEBUG_LOOPS_TO_COMPLETE = 4;
#endif

constexpr double time_step = 0.05;
constexpr int pred_horizon = 40;

// void chatterCallback(const bound_est::ConeMap &m)
// {
//   ROS_INFO("I heard: [%d]", m.cones);
// }

int main(int argc, char *argv[])
{
    //Configure ros
    ros::init(argc, argv, "boundary_est");

    //Configure visualisation
    #ifdef VISUALISE
        auto visualisation = std::make_shared<Visualisation>();
    #else
        auto visualisation = nullptr;
    #endif

    //Setup essential configuration variables
    //Configure track
    std::unique_ptr<Track> track = std::make_unique<Track>(visualisation);
    std::unique_ptr<MPCController> mpcc = std::make_unique<MPCController>(pred_horizon, time_step, visualisation);

    // //Configure ros messages
    // rosbag::Bag cone_data;
    // cone_data.open("/home/dm501/catkin_ws/src/bound_est/src/resources/cone_files/KartingGenk.bag");
    // rosbag::View view(cone_data);
    // std::vector<std::string> topics;
    // //topics.push_back(std::string("carpos")); //Testing topic, remove in final
    // topics.push_back(std::string("/map"));
    // for (rosbag::MessageInstance const m: view)
    // {
    //     auto s = m.instantiate<std_msgs::String>();
    //     if (s != NULL)
    //         std::cout << s->data << std::endl;

    //     std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
    //     if (i != NULL)
    //         std::cout << i->data << std::endl;
    //     // auto conePos = m.instantiate<ConePosBag>();
    //     // if (conePos != nullptr) std::cout << "x: "<<conePos->x<<", y: "<<conePos->y<<", and colour :"<<conePos->color<< std::endl;
    //     // if (conePos == nullptr) std::cout<<"Panic"<<std::endl;
    // }

    //Angle car to be pointing forward at beginning of race
    auto car = track->getCar();
    car->setPosition({{0,0}, 0.1});
    // LEFT
    track->addCone(0.00, 1.50, BoundPos::undefined);
    track->addCone(1.85, 1.50, BoundPos::undefined);
    track->addCone(3.73, 1.50, BoundPos::undefined);
    track->addCone(5.60, 1.50, BoundPos::undefined);
    track->addCone(7.47, 1.50, BoundPos::undefined);
    track->addCone(9.31, 1.54, BoundPos::undefined);
    track->addCone(11.16, 1.59, BoundPos::undefined); 
    track->addCone(13.03, 1.57, BoundPos::undefined);
    track->addCone(14.99, 1.39, BoundPos::undefined);
    track->addCone(16.94, 1.21, BoundPos::undefined);
    track->addCone(18.71, 1.40, BoundPos::undefined);
    track->addCone(20.17, 2.26, BoundPos::undefined);
    track->addCone(21.33, 3.72, BoundPos::undefined);
    track->addCone(22.33, 5.54, BoundPos::undefined);
    track->addCone(23.27, 7.47, BoundPos::undefined);
    track->addCone(24.27, 9.26, BoundPos::undefined);
    track->addCone(25.45, 10.68, BoundPos::undefined);
    track->addCone(26.94, 11.49, BoundPos::undefined);
    track->addCone(28.76, 11.57, BoundPos::undefined);
    track->addCone(30.66, 11.50, BoundPos::undefined);
    track->addCone(32.52, 11.50, BoundPos::undefined);
    track->addCone(34.39, 11.50, BoundPos::undefined);
    track->addCone(36.26, 11.50, BoundPos::undefined);
    track->addCone(38.13, 11.50, BoundPos::undefined);
    track->addCone(40.00, 11.50, BoundPos::undefined);

    // RIGHT
    track->addCone(0.00, -1.50, BoundPos::undefined);
    track->addCone(1.85, -1.50, BoundPos::undefined);
    track->addCone(3.73, -1.50, BoundPos::undefined);
    track->addCone(5.60, -1.50, BoundPos::undefined);
    track->addCone(7.47, -1.50, BoundPos::undefined);
    track->addCone(9.31, -1.45, BoundPos::undefined);
    track->addCone(11.16, -1.40, BoundPos::undefined); 
    track->addCone(13.03, -1.43, BoundPos::undefined);
    track->addCone(14.99, -1.6, BoundPos::undefined);
    track->addCone(16.94, -1.78, BoundPos::undefined);
    track->addCone(18.71, -1.60, BoundPos::undefined);
    track->addCone(20.17, -0.73, BoundPos::undefined);
    track->addCone(21.33, 0.72, BoundPos::undefined);
    track->addCone(22.33, 2.54, BoundPos::undefined);
    track->addCone(23.27, 4.47, BoundPos::undefined);
    track->addCone(24.27, 6.26, BoundPos::undefined);
    track->addCone(25.45, 7.68, BoundPos::undefined);
    track->addCone(26.94, 8.49, BoundPos::undefined);
    track->addCone(28.76, 8.57, BoundPos::undefined);
    track->addCone(30.66, 8.50, BoundPos::undefined);
    track->addCone(32.52, 8.50, BoundPos::undefined);
    track->addCone(34.39, 8.50, BoundPos::undefined);
    track->addCone(36.26, 8.50, BoundPos::undefined);
    track->addCone(38.13, 8.50, BoundPos::undefined);
    track->addCone(40.00, 8.50, BoundPos::undefined);

    std::cout<<"Entering loop"<<std::endl;
    auto ros_refresh_timer = std::chrono::high_resolution_clock::now();

    #if defined(DEBUG_LOOPS) || defined(DEBUG_SLOW)
        int loops_completed = 0;
    #endif

    //Enter refresh loop
    while (1)
    {
        //TEST
        //ETEST
        //MAIN LOOP
        //Collect IMU data
        //Check critical conditions 
        //If outside track constraints - error state
        //Update track - read-in data if available
        //BAG READ IN BLOCKING CALL
        /*
        *ros::NodeHandle cone_reader;
        *ros::LoopRate = 
        *ros::Subscriber cone_sub = cone_reader.subscribe("/map", 1000, chatterCallback);
        *ros::spinOnce();
        */
        //Process track section
        track->processNextSection();
        //MPCC
        auto car = track->getCar();
        auto control = mpcc->solve(*car, *track);
        //Update car outputs
        car->updateCar(control, time_step);

        #ifdef VISUALISE
            visualisation->showCar(car->getPosition());
            visualisation->showCarDirection(car->getPosition());
        #endif

        //Check metadata (laps done, goal achievement, etc.)
        //Update visualisation
        #ifdef VISUALISE    
            if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-ros_refresh_timer)).count() > ROS_REFRESH_TIME_MS)
            {
                    visualisation->refreshRosOutput();
                    ros_refresh_timer = std::chrono::high_resolution_clock::now();
            }
        #endif

        #ifdef DEBUG_SLOW
            usleep(500000);
            std::cout<<"Loops completed: "<<++loops_completed<<std::endl;
        #endif

        #ifdef DEBUG_LOOPS
        if (++loops_completed>=DEBUG_LOOPS_TO_COMPLETE) 
        {
            std::cout<<"Completed request number of loops"<<std::endl;
            break;
        }
        #endif
        #ifdef DEBUG
            reset_logs = false;
        #endif
    }
   
 } 
