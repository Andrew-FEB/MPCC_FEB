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
    std::cerr<<"ROS CONFIGURED";
    //Configure visualisation
    #ifdef VISUALISE
        auto visualisation = std::make_shared<Visualisation>();
    #else
        auto visualisation = nullptr;
    #endif
    std::cerr<<"VISUALISATION CONFIGURED";
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
    std::cerr<<"TRACK CONFIGURED";
    std::unique_ptr<MPCController> mpcc = std::make_unique<MPCController>(visualisation, *track);
    std::cerr<<"MPCC CONFIGURED";
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
    car->setPosition({{-3.88, 14.05}, -0.9427});
    std::vector<Coord> left_coords{{-0.06, 16.21},
{1.84, 13.44},
{3.39, 10.11},
{4.7, 6.57},
{5.9, 3.16},
{7.1, 0.21},
{8.43, -1.94},
{10.01, -2.96},
{11.95, -2.57},
{14.25, -1.05},
{16.83, 1.03},
{19.64, 3.09},
{22.6, 4.53},
{25.65, 4.94},
{28.68, 4.43},
{31.6, 3.25},
{34.32, 1.62},
{36.76, -0.23},
{38.94, -2.27},
{40.9, -4.5},
{42.71, -6.91},
{44.41, -9.52},
{46.07, -12.29},
{47.94, -14.61},
{50.37, -15.64},
{53.28, -15.36},
{56.41, -14.2},
{59.5, -12.6},
{62.29, -10.95},
{64.75, -9.22},
{66.91, -7.25},
{68.8, -4.85},
{70.14, -2.07},
{70.46, 0.89},
{69.54, 3.8},
{67.7, 6.34},
{65.3, 8.33},
{62.74, 10.04},
{60.46, 11.85},
{58.83, 14.11},
{57.72, 16.89},
{56.83, 20.15},
{55.91, 23.77},
{54.9, 27.45},
{53.76, 30.84},
{52.44, 33.61},
{50.93, 35.44},
{49.17, 35.99},
{47.15, 35.14},
{44.92, 33.25},
{42.54, 30.77},
{40.05, 28.14},
{37.52, 25.8},
{34.98, 24.19},
{32.5, 23.7},
{30.09, 24.3},
{27.75, 25.82},
{25.49, 28.05},
{23.3, 30.8},
{21.19, 33.89},
{19.16, 37.11},
{17.21, 40.28},
{15.28, 43.11},
{13.24, 45.19},
{10.95, 46.14},
{8.35, 45.79},
{5.55, 44.5},
{2.69, 42.68},
{-0.12, 40.75},
{-2.75, 38.99},
{-5.16, 37.26},
{-7.32, 35.27},
{-9.16, 32.81},
{-10.32, 29.97},
{-10.35, 26.96},
{-9.19, 24.1},
{-7.24, 21.71},
{-4.87, 19.91},
{-2.37, 18.25},
{-0.06, 16.21}};

    int left_coords_index{0};
    int right_coords_index{0};
    std::vector<Coord> right_coords{{-7.73, 11.9},
{-5.24, 8.59},
{-2.98, 4.77},
{-0.87, 0.74},
{1.14, -3.21},
{3.14, -6.77},
{5.19, -9.64},
{7.34, -11.53},
{9.68, -12.13},
{12.27, -11.21},
{15.12, -9.15},
{18.23, -6.75},
{21.62, -4.82},
{25.24, -4.08},
{28.92, -4.59},
{32.4, -6.08},
{35.4, -8.26},
{37.87, -10.88},
{40.05, -13.72},
{42.24, -16.58},
{44.73, -19.26},
{47.74, -21.54},
{51.12, -23.13},
{54.6, -23.72},
{57.97, -23.1},
{61.2, -21.52},
{64.31, -19.34},
{67.31, -16.94},
{70.23, -14.61},
{72.98, -12.27},
{75.45, -9.72},
{77.53, -6.74},
{78.98, -3.35},
{79.5, 0.23},
{78.89, 3.8},
{77.33, 7.15},
{75.08, 10.06},
{72.42, 12.57},
{69.65, 14.89},
{67.07, 17.25},
{64.84, 19.82},
{62.9, 22.72},
{61.13, 26.03},
{59.43, 29.84},
{57.73, 34.04},
{55.96, 38.15},
{54.08, 41.67},
{52.01, 44.13},
{49.7, 45.01},
{47.13, 44.0},
{44.34, 41.58},
{41.44, 38.5},
{38.5, 35.54},
{35.63, 33.43},
{32.91, 32.91},
{30.32, 34.04},
{27.8, 36.38},
{25.26, 39.5},
{22.61, 42.95},
{19.78, 46.28},
{16.68, 49.06},
{13.32, 51.01},
{9.83, 52.11},
{6.32, 52.32},
{2.92, 51.63},
{-0.34, 50.2},
{-3.48, 48.25},
{-6.52, 46.03},
{-9.47, 43.76},
{-12.27, 41.46},
{-14.83, 38.95},
{-17.02, 36.08},
{-18.67, 32.79},
{-19.47, 29.24},
{-19.15, 25.6},
{-17.81, 22.16},
{-15.74, 19.21},
{-13.2, 16.8},
{-10.44, 14.52},
{-7.73, 11.9}};

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
        int batch_size {3};
        for (int i = 0; i<batch_size; i++)
        {
            if (left_coords_index>=left_coords.size() || right_coords_index>=left_coords.size()) break;
            track->addCone(left_coords[left_coords_index].x, left_coords[left_coords_index].y , BoundPos::left);
            track->addCone(right_coords[right_coords_index].x, right_coords[right_coords_index].y , BoundPos::right);
            left_coords_index++,
            right_coords_index++;
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
            usleep(10000);
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
