#include <memory>
#include <iostream>
#include <chrono>
#include <string>
#include <time.h>

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
    #undef VISUALISE_RQT
#endif
#ifndef VISUALISE
    #undef VISUALISE_RQT
#endif

constexpr long ROS_REFRESH_TIME_MS = 1000;

//Configure globals
#ifdef DEBUG
    bool reset_logs = true;
    constexpr int DEBUG_LOOPS_TO_COMPLETE = 60;
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
    // cone_data.open("/home/dada/catkin_ws/src/bound_est/src/resources/cone_files/KartingGenk.bag");
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
    // LEFT
    track->addCone(0.00, 2.00, BoundPos::left);
    track->addCone(1.85, 2.00, BoundPos::undefined);
    track->addCone(3.73, 2.00, BoundPos::undefined);
    track->addCone(5.60, 2.00, BoundPos::left);
    track->addCone(7.47, 2.00, BoundPos::undefined);
    track->addCone(9.31, 2.04, BoundPos::undefined);
    track->addCone(11.16, 2.09, BoundPos::undefined); 
    track->addCone(13.03, 2.06, BoundPos::undefined);
    track->addCone(14.99, 1.90, BoundPos::undefined);
    track->addCone(16.94, 1.72, BoundPos::left);
    track->addCone(18.71, 1.9, BoundPos::undefined);
    track->addCone(20.17, 2.76, BoundPos::undefined);
    track->addCone(21.33, 4.22, BoundPos::left);
    track->addCone(22.33, 6.04, BoundPos::undefined);
    track->addCone(23.27, 8.47, BoundPos::undefined);
    track->addCone(24.27, 9.76, BoundPos::undefined);
    track->addCone(25.45, 11.18, BoundPos::undefined);
    track->addCone(26.94, 11.99, BoundPos::undefined);
    track->addCone(28.76, 12.07, BoundPos::undefined);
    track->addCone(30.66, 12.00, BoundPos::undefined);
    track->addCone(32.52, 12.00, BoundPos::undefined);
    track->addCone(34.39, 12.00, BoundPos::left);
    track->addCone(36.26, 12.00, BoundPos::undefined);
    track->addCone(38.13, 12.00, BoundPos::undefined);
    track->addCone(40.00, 12.00, BoundPos::undefined);
    track->addCone(42.52, 12.00, BoundPos::undefined);
    track->addCone(44.39, 12.00, BoundPos::undefined);
    track->addCone(46.26, 12.00, BoundPos::left);
    track->addCone(48.13, 12.00, BoundPos::undefined);
    track->addCone(50.00, 12.00, BoundPos::undefined);

    // RIGHT
    track->addCone(0.00, -2.00, BoundPos::undefined);
    track->addCone(1.85, -2.00, BoundPos::undefined);
    track->addCone(3.73, -2.00, BoundPos::undefined);
    track->addCone(5.60, -2.00, BoundPos::undefined);
    track->addCone(7.47, -2.00, BoundPos::undefined);
    track->addCone(9.31, -1.96, BoundPos::right);
    track->addCone(11.16, -1.91, BoundPos::undefined); 
    track->addCone(13.03, -1.93, BoundPos::undefined);
    track->addCone(14.99, -2.11, BoundPos::undefined);
    track->addCone(16.94, -2.28, BoundPos::undefined);
    track->addCone(18.71, -2.10, BoundPos::undefined);
    track->addCone(20.17, -1.23, BoundPos::undefined);
    track->addCone(21.33, 0.22, BoundPos::undefined);
    track->addCone(22.33, 2.04, BoundPos::undefined);
    track->addCone(23.27, 3.47, BoundPos::undefined);
    track->addCone(24.27, 5.26, BoundPos::undefined);
    track->addCone(25.45, 7.18, BoundPos::right);
    track->addCone(26.94, 7.49, BoundPos::undefined);
    track->addCone(28.76, 8.07, BoundPos::undefined);
    track->addCone(30.66, 8.00, BoundPos::undefined);
    track->addCone(32.52, 8.00, BoundPos::undefined);
    track->addCone(34.39, 8.00, BoundPos::undefined);
    track->addCone(36.26, 8.00, BoundPos::undefined);
    track->addCone(38.13, 8.00, BoundPos::undefined);
    track->addCone(40.00, 8.00, BoundPos::right);
    track->addCone(42.52, 8.00, BoundPos::undefined);
    track->addCone(44.39, 8.00, BoundPos::undefined);
    track->addCone(46.26, 8.00, BoundPos::undefined);
    track->addCone(48.13, 8.00, BoundPos::undefined);
    track->addCone(50.00, 8.00, BoundPos::undefined);

    std::cout<<"Entering loop"<<std::endl;
    auto ros_refresh_timer = std::chrono::high_resolution_clock::now();

    #if defined(DEBUG_LOOPS) || defined(DEBUG_SLOW)
        int loops_completed = 0;
    #endif

    bool continue_driving{false};
    //Enter refresh loop
    while (1)
    {
        #ifdef VISUALISE_RQT
            // Recording starting time
            auto start = clock();
        #endif
        //TEST
        //ETEST
        //MAIN LOOP
        //Collect IMU data
        //Check critical conditions 
        continue_driving = track->carIsInsideTrack();
        //Update track - read-in data if available
        //BAG READ IN BLOCKING CALL
        /*
        *ros::NodeHandle cone_reader;
        *ros::LoopRate = 
        *ros::Subscriber cone_sub = cone_reader.subscribe("/map", 1000, chatterCallback);
        *ros::spinOnce();
        */
        //Process track section
        if (!track->trackIsComplete()) track->processNextSection();
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
            usleep(100000);
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

        #ifdef VISUALISE_RQT
            //Recording finish time
            auto finish = clock();
            auto solve_time = (((float)(finish-start))/CLOCKS_PER_SEC) * 1000; // in ms
            visualisation->plotSolveTime(solve_time);
        #endif
    }
   
 } 
