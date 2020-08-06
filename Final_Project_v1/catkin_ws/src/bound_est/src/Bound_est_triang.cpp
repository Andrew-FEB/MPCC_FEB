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
    constexpr int DEBUG_LOOPS_TO_COMPLETE = 65;
#endif

constexpr double time_step = 0.05;
constexpr int pred_horizon = 30;

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
    car->setPosition({{0, 13.5}, -0.7861});
    std::vector<Coord> left_coords{{1.5, 18.0},
{4.24, 17.44},
{6.41, 15.24},
{8.15, 11.88},
{9.55, 7.84},
{10.73, 3.6},
{11.8, -0.38},
{12.88, -3.6},
{14.08, -5.59},
{15.5, -5.88},
{17.21, -4.41},
{19.16, -1.72},
{21.31, 1.57},
{23.6, 4.86},
{26.0, 7.54},
{28.45, 8.99},
{30.9, 8.77},
{33.32, 7.14},
{35.64, 4.53},
{37.83, 1.37},
{39.83, -1.88},
{41.6, -4.83},
{43.22, -7.49},
{44.81, -10.17},
{46.54, -13.02},
{48.7, -15.14},
{51.49, -15.68},
{54.69, -14.94},
{57.99, -13.43},
{61.09, -11.68},
{63.82, -9.91},
{66.21, -7.95},
{68.3, -5.58},
{69.91, -2.73},
{70.5, 0.37},
{69.71, 3.46},
{67.85, 6.18},
{65.34, 8.3},
{62.65, 10.1},
{60.29, 12.04},
{58.63, 14.5},
{57.53, 17.53},
{56.62, 21.01},
{55.69, 24.77},
{54.64, 28.46},
{53.4, 31.75},
{51.91, 34.31},
{50.09, 35.81},
{47.88, 35.96},
{45.32, 34.88},
{42.52, 32.96},
{39.56, 30.58},
{36.56, 28.11},
{33.61, 25.94},
{30.81, 24.45},
{28.26, 24.01},
{26.03, 24.87},
{24.06, 26.82},
{22.29, 29.61},
{20.64, 32.95},
{19.05, 36.57},
{17.46, 40.21},
{15.76, 43.53},
{13.85, 46.11},
{11.62, 47.51},
{8.99, 47.46},
{6.07, 46.26},
{3.04, 44.4},
{0.06, 42.36},
{-2.72, 40.53},
{-5.26, 38.71},
{-7.52, 36.56},
{-9.41, 33.85},
{-10.45, 30.82},
{-10.1, 27.86},
{-8.59, 24.98},
{-6.55, 22.03},
{-4.23, 19.37},
{-1.56, 17.82},
{1.5, 18.0}
};
    int left_coords_index{0};
    int right_coords_index{0};
    std::vector<Coord> right_coords{{-1.5, 9.0},
{1.19, 5.63},
{3.57, 1.51},
{5.73, -2.92},
{7.73, -7.24},
{9.64, -11.01},
{11.53, -13.79},
{13.46, -15.16},
{15.52, -14.69},
{17.74, -12.34},
{20.13, -8.87},
{22.66, -5.1},
{25.33, -1.9},
{28.14, -0.09},
{31.05, -0.35},
{33.95, -2.42},
{36.7, -5.68},
{39.15, -9.51},
{41.16, -13.31},
{42.91, -16.68},
{45.07, -19.57},
{48.11, -21.89},
{51.74, -23.36},
{55.47, -23.67},
{58.98, -22.71},
{62.29, -20.84},
{65.45, -18.46},
{68.52, -15.97},
{71.49, -13.57},
{74.23, -11.06},
{76.62, -8.19},
{78.46, -4.83},
{79.43, -1.16},
{79.22, 2.59},
{77.89, 6.17},
{75.72, 9.34},
{73.04, 12.03},
{70.15, 14.47},
{67.41, 16.91},
{65.03, 19.59},
{62.97, 22.6},
{61.13, 26.01},
{59.42, 29.9},
{57.73, 34.13},
{55.96, 38.24},
{54.0, 41.74},
{51.75, 44.16},
{49.1, 45.01},
{46.01, 44.07},
{42.61, 41.87},
{39.08, 39.03},
{35.57, 36.18},
{32.25, 33.95},
{29.29, 32.96},
{26.83, 33.78},
{24.77, 36.23},
{22.86, 39.72},
{20.84, 43.65},
{18.47, 47.41},
{15.54, 50.44},
{12.15, 52.53},
{8.52, 53.63},
{4.88, 53.66},
{1.4, 52.64},
{-1.93, 50.85},
{-5.14, 48.6},
{-8.24, 46.21},
{-11.23, 43.84},
{-13.99, 41.33},
{-16.41, 38.48},
{-18.33, 35.14},
{-19.39, 31.5},
{-19.27, 27.78},
{-18.0, 24.2},
{-15.97, 20.91},
{-13.49, 18.01},
{-10.67, 15.49},
{-7.64, 13.34},
{-4.5, 11.39},
{-1.5, 9.0}};
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
        for (int i = 0; i<3; i++)
        {
            left_coords_index++,
            right_coords_index++;
            track->addCone(left_coords[left_coords_index].x, left_coords[left_coords_index].y , BoundPos::left);
            track->addCone(right_coords[right_coords_index].x, right_coords[right_coords_index].y , BoundPos::right);
        }
        //BAG READ IN BLOCKING CALL
        /*
        *ros::NodeHandle cone_reader;
        *ros::LoopRate = 
        *ros::Subscriber cone_sub = cone_reader.subscribe("/map", 1000, chatterCallback);
        *ros::spinOnce();
        */
       //TEST
        if (!track->trackIsComplete()) track->processNextSection();
        mpcc->solve();  
        //ETEST
        /*
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
       }*/
        #if defined(DEBUG_LOOPS) || defined(DEBUG_SLOW) ||defined(DEBUG_SLOWAFTERLOOPS) ||defined(TIME_LOG)
            loops_completed++;
        #endif
        #ifdef DEBUG_SLOW
            usleep(100000);
            std::cout<<"Loops completed: "<<loops_completed<<std::endl;
        #endif

        #ifdef DEBUG_SLOWAFTERLOOPS
            std::cout<<"Loops completed: "<<loops_completed<<std::endl;
            if (loops_completed>=DEBUG_LOOPS_TO_COMPLETE)
            {
                usleep(1000000);
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
