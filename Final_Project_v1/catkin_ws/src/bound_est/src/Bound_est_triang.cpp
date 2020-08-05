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
#include "boundaryLogger.h"

#ifndef DEBUG
    #undef DDEBUG_LOOPS
    #undef DDEBUG_SLOW
    #undef DDEBUG_SLOWAFTERLOOPS
    #undef VISUALISE
    #undef VISUALISE_RQT
#endif
#ifndef VISUALISE
    #undef VISUALISE_RQT
#endif

//Configure globals
bool counter_clockwise_track = false;
bool clockwise_track = false;
#if defined(DEBUG) || defined(TIME_LOG)
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
    //Check track orientation input argument
    if (argc>=1)
    {
        if (std::string(argv[1])=="cw")
        {
            std::cout<<"Clockwise track declared in compiler statement. Adjusting track framing."<<std::endl;
            clockwise_track = true;
        } 
        else if (std::string(argv[1])=="ccw") 
        {
            std::cout<<"Counter clockwise track declared in compiler statement. Adjusting track framing."<<std::endl;
            counter_clockwise_track = true;
        }
    }

    //Configure ros
    ros::init(argc, argv, "boundary_est");

    //Configure visualisation
    #ifdef VISUALISE
        auto visualisation = std::make_shared<Visualisation>();
    #else
        auto visualisation = nullptr;
    #endif

    //Configure time logs
    #ifdef TIME_LOG
    std::unique_ptr<BoundaryLogger> time_log = std::make_unique<BoundaryLogger>("TIME_LOG", "Timing logs", reset_logs);
    std::stringstream ss;
    time_log->write(ss<<"Logged in microseconds");
    time_log->write(ss<<"Loop  Boundary_check  Track_process   MPCC    Total");
    bool keep_printing_time{true};
    constexpr int NUM_TIMING_LOOPS = 5000;
    auto function_start = std::chrono::high_resolution_clock::now();
    int64_t section_process_time;
    int64_t mpcc_time;
    int64_t car_check_time;
    #endif

    //Setup essential configuration variables
    //Configure track
    std::unique_ptr<Track> track = std::make_unique<Track>(visualisation);
    std::unique_ptr<MPCController> mpcc = std::make_unique<MPCController>(pred_horizon, time_step, visualisation, *track);

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
    car->setPosition({{0, 15}, -0.98});
    //LEFT
    track->addCone(1.5, 19.5, BoundPos::left);
    track->addCone(3.56, 16.65, BoundPos::left);
    track->addCone(5.32, 13.22, BoundPos::left);
    track->addCone(6.84, 9.45, BoundPos::left);
    track->addCone(8.18, 5.6, BoundPos::left);
    track->addCone(9.41, 1.91, BoundPos::left);
    track->addCone(10.58, -1.36, BoundPos::left);
    track->addCone(11.77, -3.96, BoundPos::left);
    track->addCone(13.02, -5.65, BoundPos::left);
    track->addCone(14.4, -6.17, BoundPos::left);
    track->addCone(15.97, -5.31, BoundPos::left);
    track->addCone(17.74, -3.22, BoundPos::left);
    track->addCone(19.68, -0.38, BoundPos::left);
    track->addCone(21.76, 2.74, BoundPos::left);
    track->addCone(23.96, 5.66, BoundPos::left);
    track->addCone(26.23, 7.91, BoundPos::left);
    track->addCone(28.56, 9.01, BoundPos::left);
    track->addCone(30.91, 8.64, BoundPos::left);
    track->addCone(33.24, 7.03, BoundPos::left);
    track->addCone(35.49, 4.56, BoundPos::left);
    track->addCone(37.63, 1.59, BoundPos::left);
    track->addCone(39.59, -1.51, BoundPos::left);
    track->addCone(41.34, -4.38, BoundPos::left);
    track->addCone(42.92, -6.98, BoundPos::left);
    track->addCone(44.45, -9.55, BoundPos::left);
    track->addCone(46.07, -12.31, BoundPos::left);
    track->addCone(48.01, -14.67, BoundPos::left);
    track->addCone(50.53, -15.67, BoundPos::left);
    track->addCone(53.52, -15.31, BoundPos::left);
    track->addCone(56.71, -14.08, BoundPos::left);
    track->addCone(59.82, -12.42, BoundPos::left);
    track->addCone(62.62, -10.74, BoundPos::left);
    track->addCone(65.07, -8.96, BoundPos::left);
    track->addCone(67.23, -6.89, BoundPos::left);
    track->addCone(69.1, -4.37, BoundPos::left);
    track->addCone(70.31, -1.49, BoundPos::left);
    track->addCone(70.34, 1.53, BoundPos::left);
    track->addCone(69.15, 4.43, BoundPos::left);
    track->addCone(67.15, 6.9, BoundPos::left);
    track->addCone(64.73, 8.85, BoundPos::left);
    track->addCone(62.19, 10.59, BoundPos::left);
    track->addCone(59.85, 12.48, BoundPos::left);
    track->addCone(57.94, 14.82, BoundPos::left);
    track->addCone(56.46, 17.73, BoundPos::left);
    track->addCone(55.34, 21.12, BoundPos::left);
    track->addCone(54.45, 24.73, BoundPos::left);
    track->addCone(53.63, 28.27, BoundPos::left);
    track->addCone(52.75, 31.46, BoundPos::left);
    track->addCone(51.66, 34.02, BoundPos::left);
    track->addCone(50.22, 35.66, BoundPos::left);
    track->addCone(48.3, 36.12, BoundPos::left);
    track->addCone(45.9, 35.44, BoundPos::left);
    track->addCone(43.15, 33.93, BoundPos::left);
    track->addCone(40.15, 31.87, BoundPos::left);
    track->addCone(37.04, 29.57, BoundPos::left);
    track->addCone(33.93, 27.33, BoundPos::left);
    track->addCone(30.94, 25.45, BoundPos::left);
    track->addCone(28.2, 24.22, BoundPos::left);
    track->addCone(25.82, 23.94, BoundPos::left);
    track->addCone(23.89, 24.86, BoundPos::left);
    track->addCone(22.36, 26.86, BoundPos::left);
    track->addCone(21.08, 29.68, BoundPos::left);
    track->addCone(19.93, 33.05, BoundPos::left);
    track->addCone(18.77, 36.72, BoundPos::left);
    track->addCone(17.47, 40.42, BoundPos::left);
    track->addCone(15.97, 43.97, BoundPos::left);
    track->addCone(14.25, 47.22, BoundPos::left);
    track->addCone(12.33, 50.06, BoundPos::left);
    track->addCone(10.22, 52.37, BoundPos::left);
    track->addCone(7.92, 54.03, BoundPos::left);
    track->addCone(5.44, 54.92, BoundPos::left);
    track->addCone(2.8, 54.92, BoundPos::left);
    track->addCone(0.01, 54.1, BoundPos::left);
    track->addCone(-2.83, 52.7, BoundPos::left);
    track->addCone(-5.66, 51.0, BoundPos::left);
    track->addCone(-8.41, 49.25, BoundPos::left);
    track->addCone(-11.01, 47.54, BoundPos::left);
    track->addCone(-13.4, 45.68, BoundPos::left);
    track->addCone(-15.52, 43.45, BoundPos::left);
    track->addCone(-17.21, 40.75, BoundPos::left);
    track->addCone(-17.99, 37.79, BoundPos::left);
    track->addCone(-17.46, 34.83, BoundPos::left);
    track->addCone(-15.9, 32.06, BoundPos::left);
    track->addCone(-13.83, 29.66, BoundPos::left);
    track->addCone(-11.48, 27.65, BoundPos::left);
    track->addCone(-8.92, 25.97, BoundPos::left);
    track->addCone(-6.23, 24.54, BoundPos::left);
    track->addCone(-3.51, 23.19, BoundPos::left);
    track->addCone(-0.89, 21.61, BoundPos::left);
    track->addCone(1.5, 19.5, BoundPos::left);
    //RIGHT
    track->addCone(-1.5, 10.5, BoundPos::right);
    track->addCone(0.86, 6.84, BoundPos::right);
    track->addCone(3.01, 2.67, BoundPos::right);
    track->addCone(4.98, -1.71, BoundPos::right);
    track->addCone(6.84, -5.96, BoundPos::right);
    track->addCone(8.63, -9.77, BoundPos::right);
    track->addCone(10.39, -12.81, BoundPos::right);
    track->addCone(12.19, -14.77, BoundPos::right);
    track->addCone(14.07, -15.32, BoundPos::right);
    track->addCone(16.08, -14.17, BoundPos::right);
    track->addCone(18.24, -11.53, BoundPos::right);
    track->addCone(20.54, -8.09, BoundPos::right);
    track->addCone(22.98, -4.56, BoundPos::right);
    track->addCone(25.54, -1.64, BoundPos::right);
    track->addCone(28.22, -0.06, BoundPos::right);
    track->addCone(31.0, -0.36, BoundPos::right);
    track->addCone(33.78, -2.29, BoundPos::right);
    track->addCone(36.42, -5.32, BoundPos::right);
    track->addCone(38.8, -8.93, BoundPos::right);
    track->addCone(40.8, -12.59, BoundPos::right);
    track->addCone(42.48, -15.91, BoundPos::right);
    track->addCone(44.37, -18.79, BoundPos::right);
    track->addCone(47.01, -21.2, BoundPos::right);
    track->addCone(50.33, -22.92, BoundPos::right);
    track->addCone(53.9, -23.7, BoundPos::right);
    track->addCone(57.36, -23.3, BoundPos::right);
    track->addCone(60.61, -21.89, BoundPos::right);
    track->addCone(63.7, -19.83, BoundPos::right);
    track->addCone(66.67, -17.47, BoundPos::right);
    track->addCone(69.57, -15.13, BoundPos::right);
    track->addCone(72.35, -12.83, BoundPos::right);
    track->addCone(74.89, -10.37, BoundPos::right);
    track->addCone(77.07, -7.52, BoundPos::right);
    track->addCone(78.7, -4.23, BoundPos::right);
    track->addCone(79.47, -0.7, BoundPos::right);
    track->addCone(79.14, 2.86, BoundPos::right);
    track->addCone(77.81, 6.26, BoundPos::right);
    track->addCone(75.78, 9.31, BoundPos::right);
    track->addCone(73.3, 11.94, BoundPos::right);
    track->addCone(70.58, 14.29, BoundPos::right);
    track->addCone(67.81, 16.53, BoundPos::right);
    track->addCone(65.17, 18.85, BoundPos::right);
    track->addCone(62.74, 21.42, BoundPos::right);
    track->addCone(60.61, 24.43, BoundPos::right);
    track->addCone(58.81, 28.02, BoundPos::right);
    track->addCone(57.29, 32.04, BoundPos::right);
    track->addCone(55.87, 36.09, BoundPos::right);
    track->addCone(54.41, 39.79, BoundPos::right);
    track->addCone(52.73, 42.76, BoundPos::right);
    track->addCone(50.69, 44.61, BoundPos::right);
    track->addCone(48.13, 44.99, BoundPos::right);
    track->addCone(45.07, 43.92, BoundPos::right);
    track->addCone(41.65, 41.87, BoundPos::right);
    track->addCone(38.07, 39.29, BoundPos::right);
    track->addCone(34.48, 36.65, BoundPos::right);
    track->addCone(31.07, 34.41, BoundPos::right);
    track->addCone(28.0, 33.05, BoundPos::right);
    track->addCone(25.45, 33.02, BoundPos::right);
    track->addCone(23.51, 34.62, BoundPos::right);
    track->addCone(21.99, 37.52, BoundPos::right);
    track->addCone(20.62, 41.26, BoundPos::right);
    track->addCone(19.15, 45.35, BoundPos::right);
    track->addCone(17.31, 49.33, BoundPos::right);
    track->addCone(14.98, 52.87, BoundPos::right);
    track->addCone(12.25, 55.89, BoundPos::right);
    track->addCone(9.2, 58.31, BoundPos::right);
    track->addCone(5.93, 60.08, BoundPos::right);
    track->addCone(2.54, 61.11, BoundPos::right);
    track->addCone(-0.88, 61.35, BoundPos::right);
    track->addCone(-4.22, 60.73, BoundPos::right);
    track->addCone(-7.47, 59.36, BoundPos::right);
    track->addCone(-10.61, 57.47, BoundPos::right);
    track->addCone(-13.65, 55.29, BoundPos::right);
    track->addCone(-16.6, 53.05, BoundPos::right);
    track->addCone(-19.42, 50.78, BoundPos::right);
    track->addCone(-21.99, 48.33, BoundPos::right);
    track->addCone(-24.23, 45.52, BoundPos::right);
    track->addCone(-25.98, 42.28, BoundPos::right);
    track->addCone(-26.92, 38.79, BoundPos::right);
    track->addCone(-26.76, 35.25, BoundPos::right);
    track->addCone(-25.57, 31.82, BoundPos::right);
    track->addCone(-23.66, 28.66, BoundPos::right);
    track->addCone(-21.31, 25.86, BoundPos::right);
    track->addCone(-18.66, 23.41, BoundPos::right);
    track->addCone(-15.81, 21.28, BoundPos::right);
    track->addCone(-12.84, 19.41, BoundPos::right);
    track->addCone(-9.86, 17.63, BoundPos::right);
    track->addCone(-6.93, 15.71, BoundPos::right);
    track->addCone(-4.11, 13.41, BoundPos::right);
    track->addCone(-1.5, 10.5, BoundPos::right);

    std::cout << "Entering loop" << std::endl;

    #if defined(DEBUG_LOOPS) || defined(DEBUG_SLOW) ||defined(DEBUG_SLOWAFTERLOOPS) ||defined(TIME_LOG)
        int loops_completed = 0;
    #endif

    bool continue_driving{false};
    //Enter refresh loop
    while (1)
    {
        #if defined(VISUALISE_RQT) || defined(TIME_LOG)
            // Recording starting time
            auto start = std::chrono::high_resolution_clock::now();
        #endif

        //MAIN LOOP
        //Collect IMU data
        //Check critical conditions 
        #ifdef TIME_LOG
            if (keep_printing_time) function_start = std::chrono::high_resolution_clock::now();
        #endif
        continue_driving = track->carIsInsideTrack();
        #ifdef TIME_LOG
            if (keep_printing_time) car_check_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-function_start).count();
        #endif
        //Update track - read-in data if available
        //BAG READ IN BLOCKING CALL
        /*
        *ros::NodeHandle cone_reader;
        *ros::LoopRate = 
        *ros::Subscriber cone_sub = cone_reader.subscribe("/map", 1000, chatterCallback);
        *ros::spinOnce();
        */
       #ifdef TIME_LOG
        #endif
       if (continue_driving)
       {
            #ifdef TIME_LOG
                if (keep_printing_time) function_start = std::chrono::high_resolution_clock::now();
            #endif
            //Process track section
            if (!track->trackIsComplete()) track->processNextSection();
            #ifdef TIME_LOG
                if (keep_printing_time) section_process_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-function_start).count();
            #endif
            #ifdef TIME_LOG
                if (keep_printing_time) function_start = std::chrono::high_resolution_clock::now();
            #endif
            //MPCC
            mpcc->solve();  
            #ifdef TIME_LOG
                if (keep_printing_time) mpcc_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-function_start).count();
            #endif      
        //Check metadata (laps done, goal achievement, etc.)
       }
        #if defined(DEBUG_LOOPS) || defined(DEBUG_SLOW) ||defined(DEBUG_SLOWAFTERLOOPS) ||defined(TIME_LOG)
            loops_completed++;
        #endif
        #ifdef DEBUG_SLOW
            usleep(50000);
            std::cout<<"Loops completed: "<<loops_completed<<std::endl;
        #endif

        #ifdef DEBUG_SLOWAFTERLOOPS
            std::cout<<"Loops completed: "<<loops_completed<<std::endl;
            if (loops_completed>=DEBUG_LOOPS_TO_COMPLETE)
            {
                usleep(100000);
            }
        #endif

        #ifdef DEBUG_LOOPS
        if (loops_completed>=DEBUG_LOOPS_TO_COMPLETE) 
        {
            std::cout<<"Completed request number of loops"<<std::endl;
            break;
        }
        std::cout<<"Loops completed: "<<loops_completed<<std::endl;
        #endif
        #ifdef DEBUG
            reset_logs = false;
        #endif

        #ifdef VISUALISE_RQT
            //Recording finish time
            auto solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count(); // in ms
            visualisation->plotSolveTime(solve_time);
        #endif
        #ifdef TIME_LOG
            if (keep_printing_time) 
            {
                auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-start).count();
                time_log->write(ss<<loops_completed<<"           "<<car_check_time<<"            "<<section_process_time<<"        "<<mpcc_time<<"     "<<total_time);
            }
            if (loops_completed>=5000)
            {
                keep_printing_time = false;
            }
        #endif      
    }
   
 } 
