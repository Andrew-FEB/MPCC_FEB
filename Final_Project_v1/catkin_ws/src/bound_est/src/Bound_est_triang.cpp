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
    // cone_data.open("/home/senne/catkin_ws/src/bound_est/src/resources/cone_files/KartingGenk.bag");
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
    car->setPosition({{-0.0927, 13.0348}, -0.9198});
    //LEFT
    track->addCone(0.09, 17.71, BoundPos::left);
    track->addCone(2.43, 14.91, BoundPos::left);
    track->addCone(4.45, 11.51, BoundPos::left);
    track->addCone(6.23, 7.78, BoundPos::left);
    track->addCone(7.81, 4.0, BoundPos::left);
    track->addCone(9.28, 0.45, BoundPos::left);
    track->addCone(10.68, -2.59, BoundPos::left);
    track->addCone(12.09, -4.84, BoundPos::left);
    track->addCone(13.57, -6.03, BoundPos::left);
    track->addCone(15.19, -5.87, BoundPos::left);
    track->addCone(16.98, -4.28, BoundPos::left);
    track->addCone(18.93, -1.66, BoundPos::left);
    track->addCone(21.03, 1.46, BoundPos::left);
    track->addCone(23.25, 4.59, BoundPos::left);
    track->addCone(25.56, 7.2, BoundPos::left);
    track->addCone(27.95, 8.78, BoundPos::left);
    track->addCone(30.39, 8.89, BoundPos::left);
    track->addCone(32.83, 7.62, BoundPos::left);
    track->addCone(35.2, 5.32, BoundPos::left);
    track->addCone(37.43, 2.34, BoundPos::left);
    track->addCone(39.47, -0.96, BoundPos::left);
    track->addCone(41.26, -4.25, BoundPos::left);
    track->addCone(42.84, -7.32, BoundPos::left);
    track->addCone(44.42, -10.14, BoundPos::left);
    track->addCone(46.2, -12.65, BoundPos::left);
    track->addCone(48.35, -14.49, BoundPos::left);
    track->addCone(51.03, -15.17, BoundPos::left);
    track->addCone(54.08, -14.81, BoundPos::left);
    track->addCone(57.3, -13.71, BoundPos::left);
    track->addCone(60.48, -12.18, BoundPos::left);
    track->addCone(63.42, -10.42, BoundPos::left);
    track->addCone(66.02, -8.37, BoundPos::left);
    track->addCone(68.17, -5.94, BoundPos::left);
    track->addCone(69.71, -3.11, BoundPos::left);
    track->addCone(70.3, -0.04, BoundPos::left);
    track->addCone(69.68, 3.04, BoundPos::left);
    track->addCone(68.08, 5.77, BoundPos::left);
    track->addCone(65.87, 7.84, BoundPos::left);
    track->addCone(63.41, 9.53, BoundPos::left);
    track->addCone(61.04, 11.4, BoundPos::left);
    track->addCone(59.08, 13.97, BoundPos::left);
    track->addCone(57.56, 17.21, BoundPos::left);
    track->addCone(56.35, 20.86, BoundPos::left);
    track->addCone(55.31, 24.65, BoundPos::left);
    track->addCone(54.32, 28.29, BoundPos::left);
    track->addCone(53.26, 31.51, BoundPos::left);
    track->addCone(51.98, 34.04, BoundPos::left);
    track->addCone(50.36, 35.6, BoundPos::left);
    track->addCone(48.28, 35.92, BoundPos::left);
    track->addCone(45.75, 35.08, BoundPos::left);
    track->addCone(42.88, 33.4, BoundPos::left);
    track->addCone(39.8, 31.21, BoundPos::left);
    track->addCone(36.62, 28.84, BoundPos::left);
    track->addCone(33.46, 26.61, BoundPos::left);
    track->addCone(30.43, 24.86, BoundPos::left);
    track->addCone(27.67, 23.92, BoundPos::left);
    track->addCone(25.28, 24.12, BoundPos::left);
    track->addCone(23.33, 25.61, BoundPos::left);
    track->addCone(21.72, 28.15, BoundPos::left);
    track->addCone(20.34, 31.39, BoundPos::left);
    track->addCone(19.08, 35.02, BoundPos::left);
    track->addCone(17.82, 38.71, BoundPos::left);
    track->addCone(16.45, 42.13, BoundPos::left);
    track->addCone(14.84, 44.95, BoundPos::left);
    track->addCone(12.9, 46.84, BoundPos::left);
    track->addCone(10.51, 47.5, BoundPos::left);
    track->addCone(7.69, 46.95, BoundPos::left);
    track->addCone(4.62, 45.53, BoundPos::left);
    track->addCone(1.44, 43.6, BoundPos::left);
    track->addCone(-1.66, 41.5, BoundPos::left);
    track->addCone(-4.52, 39.37, BoundPos::left);
    track->addCone(-6.97, 37.14, BoundPos::left);
    track->addCone(-8.83, 34.75, BoundPos::left);
    track->addCone(-9.93, 32.13, BoundPos::left);
    track->addCone(-10.1, 29.2, BoundPos::left);
    track->addCone(-9.27, 26.14, BoundPos::left);
    track->addCone(-7.58, 23.47, BoundPos::left);
    track->addCone(-5.23, 21.49, BoundPos::left);
    track->addCone(-2.55, 19.76, BoundPos::left);
    track->addCone(0.09, 17.71, BoundPos::left);

    //RIGHT
    track->addCone(-0.04, 8.78, BoundPos::right);
    track->addCone(2.62, 5.39, BoundPos::right);
    track->addCone(4.89, 1.24, BoundPos::right);
    track->addCone(6.88, -3.21, BoundPos::right);
    track->addCone(8.68, -7.52, BoundPos::right);
    track->addCone(10.4, -11.22, BoundPos::right);
    track->addCone(12.14, -13.86, BoundPos::right);
    track->addCone(13.99, -14.98, BoundPos::right);
    track->addCone(16.07, -14.16, BoundPos::right);
    track->addCone(18.39, -11.58, BoundPos::right);
    track->addCone(20.9, -8.06, BoundPos::right);
    track->addCone(23.54, -4.43, BoundPos::right);
    track->addCone(26.27, -1.52, BoundPos::right);
    track->addCone(29.02, -0.15, BoundPos::right);
    track->addCone(31.76, -0.82, BoundPos::right);
    track->addCone(34.42, -3.1, BoundPos::right);
    track->addCone(36.98, -6.47, BoundPos::right);
    track->addCone(39.39, -10.38, BoundPos::right);
    track->addCone(41.62, -14.3, BoundPos::right);
    track->addCone(43.86, -17.83, BoundPos::right);
    track->addCone(46.45, -20.66, BoundPos::right);
    track->addCone(49.51, -22.56, BoundPos::right);
    track->addCone(52.86, -23.44, BoundPos::right);
    track->addCone(56.31, -23.23, BoundPos::right);
    track->addCone(59.77, -22.04, BoundPos::right);
    track->addCone(63.17, -20.15, BoundPos::right);
    track->addCone(66.5, -17.84, BoundPos::right);
    track->addCone(69.72, -15.38, BoundPos::right);
    track->addCone(72.71, -12.78, BoundPos::right);
    track->addCone(75.35, -9.92, BoundPos::right);
    track->addCone(77.5, -6.66, BoundPos::right);
    track->addCone(78.94, -3.08, BoundPos::right);
    track->addCone(79.38, 0.51, BoundPos::right);
    track->addCone(78.66, 3.87, BoundPos::right);
    track->addCone(76.98, 7.01, BoundPos::right);
    track->addCone(74.63, 9.97, BoundPos::right);
    track->addCone(71.92, 12.81, BoundPos::right);
    track->addCone(69.12, 15.58, BoundPos::right);
    track->addCone(66.52, 18.34, BoundPos::right);
    track->addCone(64.23, 21.19, BoundPos::right);
    track->addCone(62.2, 24.28, BoundPos::right);
    track->addCone(60.35, 27.74, BoundPos::right);
    track->addCone(58.63, 31.67, BoundPos::right);
    track->addCone(56.92, 35.79, BoundPos::right);
    track->addCone(55.1, 39.62, BoundPos::right);
    track->addCone(53.02, 42.7, BoundPos::right);
    track->addCone(50.57, 44.56, BoundPos::right);
    track->addCone(47.62, 44.77, BoundPos::right);
    track->addCone(44.21, 43.44, BoundPos::right);
    track->addCone(40.54, 41.12, BoundPos::right);
    track->addCone(36.78, 38.35, BoundPos::right);
    track->addCone(33.12, 35.7, BoundPos::right);
    track->addCone(29.75, 33.7, BoundPos::right);
    track->addCone(26.84, 32.93, BoundPos::right);
    track->addCone(24.58, 33.89, BoundPos::right);
    track->addCone(22.88, 36.55, BoundPos::right);
    track->addCone(21.4, 40.24, BoundPos::right);
    track->addCone(19.78, 44.29, BoundPos::right);
    track->addCone(17.67, 48.02, BoundPos::right);
    track->addCone(14.84, 50.89, BoundPos::right);
    track->addCone(11.47, 52.78, BoundPos::right);
    track->addCone(7.82, 53.66, BoundPos::right);
    track->addCone(4.17, 53.52, BoundPos::right);
    track->addCone(0.66, 52.42, BoundPos::right);
    track->addCone(-2.71, 50.6, BoundPos::right);
    track->addCone(-5.98, 48.3, BoundPos::right);
    track->addCone(-9.17, 45.76, BoundPos::right);
    track->addCone(-12.2, 43.08, BoundPos::right);
    track->addCone(-14.9, 40.25, BoundPos::right);
    track->addCone(-17.09, 37.24, BoundPos::right);
    track->addCone(-18.6, 34.01, BoundPos::right);
    track->addCone(-19.23, 30.54, BoundPos::right);
    track->addCone(-18.85, 26.85, BoundPos::right);
    track->addCone(-17.55, 23.19, BoundPos::right);
    track->addCone(-15.48, 19.91, BoundPos::right);
    track->addCone(-12.82, 17.15, BoundPos::right);
    track->addCone(-9.75, 14.85, BoundPos::right);
    track->addCone(-6.45, 12.95, BoundPos::right);
    track->addCone(-3.13, 11.14, BoundPos::right);
    track->addCone(-0.04, 8.78, BoundPos::right);

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
