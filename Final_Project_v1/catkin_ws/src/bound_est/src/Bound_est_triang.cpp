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
    // car->setPosition({{23.6, 29.6}, 0});
    // LEFT
    track->addCone(-0.14, 3.51, BoundPos::left);
    track->addCone(2.21, 3.74, BoundPos::left);
    track->addCone(4.76, 5.27, BoundPos::left);
    track->addCone(7.44, 7.15, BoundPos::left);
    track->addCone(10.24, 8.54, BoundPos::left);
    track->addCone(13.11, 9.40, BoundPos::left);
    track->addCone(16.02, 10.04, BoundPos::left);
    track->addCone(18.94, 10.76, BoundPos::left);
    track->addCone(21.83, 11.50, BoundPos::left);
    track->addCone(24.64, 12.00, BoundPos::left);
    track->addCone(27.33, 11.89, BoundPos::left);
    track->addCone(29.87, 10.95, BoundPos::left);
    track->addCone(32.22, 9.03, BoundPos::left);
    track->addCone(34.42, 6.49, BoundPos::left);
    track->addCone(36.50, 3.75, BoundPos::left);
    track->addCone(38.48, 1.26, BoundPos::left);
    track->addCone(40.42, -0.53, BoundPos::left);
    track->addCone(42.34, -1.18, BoundPos::left);
    track->addCone(44.26, -0.35, BoundPos::left);
    track->addCone(46.07, 1.71, BoundPos::left);
    track->addCone(47.54, 4.49, BoundPos::left);
    track->addCone(48.62, 7.66, BoundPos::left);
    track->addCone(49.26, 11.03, BoundPos::left);
    track->addCone(49.45, 14.43, BoundPos::left);
    track->addCone(49.15, 17.67, BoundPos::left);
    track->addCone(48.35, 20.56, BoundPos::left);
    track->addCone(47.01, 22.92, BoundPos::left);
    track->addCone(45.12, 24.57, BoundPos::left);
    track->addCone(42.70, 25.50, BoundPos::left);
    track->addCone(39.87, 25.91, BoundPos::left);
    track->addCone(36.76, 26.02, BoundPos::left);
    track->addCone(33.49, 26.04, BoundPos::left);
    track->addCone(30.17, 26.19, BoundPos::left);
    track->addCone(26.94, 26.51, BoundPos::left);
    track->addCone(23.93, 26.66, BoundPos::left);
    track->addCone(21.26, 26.29, BoundPos::left);
    track->addCone(19.07, 25.05, BoundPos::left);
    track->addCone(17.30, 22.95, BoundPos::left);
    track->addCone(15.70, 20.39, BoundPos::left);
    track->addCone(14.00, 17.78, BoundPos::left);
    track->addCone(11.98, 15.54, BoundPos::left);
    track->addCone(9.36, 14.06, BoundPos::left);
    track->addCone(6.09, 13.52, BoundPos::left);
    track->addCone(2.61, 13.45, BoundPos::left);
    track->addCone(-0.57, 13.34, BoundPos::left);
    track->addCone(-2.95, 12.63, BoundPos::left);
    track->addCone(-4.03, 10.81, BoundPos::left);
    track->addCone(-3.66, 7.98, BoundPos::left);
    track->addCone(-2.22, 5.18, BoundPos::left);
    track->addCone(-0.14, 3.51, BoundPos::left);

    // RIGHT
    track->addCone(-0.01, -2.90, BoundPos::right);
    track->addCone(2.89, -1.79, BoundPos::right);
    track->addCone(5.91, 0.43, BoundPos::right);
    track->addCone(9.13, 2.35, BoundPos::right);
    track->addCone(12.57, 3.37, BoundPos::right);
    track->addCone(16.12, 4.00, BoundPos::right);
    track->addCone(19.69, 4.81, BoundPos::right);
    track->addCone(23.17, 5.63, BoundPos::right);
    track->addCone(26.50, 5.92, BoundPos::right);
    track->addCone(29.61, 5.13, BoundPos::right);
    track->addCone(32.44, 2.94, BoundPos::right);
    track->addCone(35.04, -0.13, BoundPos::right);
    track->addCone(37.48, -3.33, BoundPos::right);
    track->addCone(39.85, -5.90, BoundPos::right);
    track->addCone(42.21, -7.09, BoundPos::right);
    track->addCone(44.61, -6.30, BoundPos::right);
    track->addCone(46.88, -3.96, BoundPos::right);
    track->addCone(48.81, -0.88, BoundPos::right);
    track->addCone(50.32, 2.67, BoundPos::right);
    track->addCone(51.40, 6.54, BoundPos::right);
    track->addCone(52.03, 10.58, BoundPos::right);
    track->addCone(52.19, 14.63, BoundPos::right);
    track->addCone(51.87, 18.55, BoundPos::right);
    track->addCone(51.04, 22.18, BoundPos::right);
    track->addCone(49.69, 25.39, BoundPos::right);
    track->addCone(47.80, 28.01, BoundPos::right);
    track->addCone(45.35, 29.90, BoundPos::right);
    track->addCone(42.36, 31.03, BoundPos::right);
    track->addCone(38.94, 31.62, BoundPos::right);
    track->addCone(35.23, 31.93, BoundPos::right);
    track->addCone(31.34, 32.23, BoundPos::right);
    track->addCone(27.41, 32.65, BoundPos::right);
    track->addCone(23.71, 32.81, BoundPos::right);
    track->addCone(20.49, 32.11, BoundPos::right);
    track->addCone(17.96, 30.10, BoundPos::right);
    track->addCone(15.91, 27.13, BoundPos::right);
    track->addCone(13.93, 23.92, BoundPos::right);
    track->addCone(11.61, 21.16, BoundPos::right);
    track->addCone(8.55, 19.54, BoundPos::right);
    track->addCone(4.76, 19.15, BoundPos::right);
    track->addCone(0.68, 19.37, BoundPos::right);
    track->addCone(-3.21, 19.55, BoundPos::right);
    track->addCone(-6.43, 19.03, BoundPos::right);
    track->addCone(-8.52, 17.16, BoundPos::right);
    track->addCone(-9.25, 13.78, BoundPos::right);
    track->addCone(-8.83, 9.51, BoundPos::right);
    track->addCone(-7.46, 5.03, BoundPos::right);
    track->addCone(-5.39, 1.02, BoundPos::right);
    track->addCone(-2.82, -1.85, BoundPos::right);
    track->addCone(-0.01, -2.90, BoundPos::right);

    std::cout << "Entering loop" << std::endl;
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

        #ifdef VISUALISE_RQT
            //Recording finish time
            auto finish = clock();
            auto solve_time = (((float)(finish-start))/CLOCKS_PER_SEC) * 1000; // in ms
            visualisation->plotSolveTime(solve_time);
        #endif
    }
   
 } 
