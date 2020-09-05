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
    constexpr int DEBUG_LOOPS_TO_COMPLETE = 873;
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
    #ifdef DEBUG
    std::cerr<<"ROS CONFIGURED"<<std::endl;
    #endif

    //Configure visualisation
    #ifdef VISUALISE
        auto visualisation = std::make_shared<Visualisation>();
    #else
        auto visualisation = nullptr;
    #endif
    #ifdef DEBUG
    std::cerr<<"VISUALISATION CONFIGURED"<<std::endl;
    #endif

    //Configure time logs
    #ifdef TIME_LOG
    std::unique_ptr<BoundaryLogger> time_log = std::make_unique<BoundaryLogger>("TIME_LOG", "Timing logs", reset_logs);
    std::stringstream ss;
    time_log->write(ss<<"Logged in microseconds");
    time_log->write(ss<<"Loop  Boundary_check  Track_process   MPCC    Total");
    auto function_start = std::chrono::high_resolution_clock::now();
    int64_t section_process_time;
    int64_t mpcc_time;
    int64_t car_check_time;
    #endif

    //Configure variables
    #if defined(DEBUG_LOOPS) || defined(DEBUG_SLOW) ||defined(DEBUG_SLOWAFTERLOOPS) ||defined(TIME_LOG)
        int loops_completed = 0;
    #endif
    bool continue_driving{false};

    //Setup essential configuration variables
    //Configure track
    std::unique_ptr<Track> track = std::make_unique<Track>(visualisation);
    #ifdef DEBUG
    std::cerr<<"TRACK CONFIGURED"<<std::endl;
    #endif
    std::unique_ptr<MPCController> mpcc = std::make_unique<MPCController>(visualisation, *track, true);
    #ifdef DEBUG
    std::cerr<<"MPCC CONFIGURED"<<std::endl;
    #endif
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
    car->setPosition({{-3.88, 14.05}, -0.9427});
    std::vector<Coord> left_coords{{-0.06, 16.22},
{2.07, 13.15},
{3.82, 9.46},
{5.33, 5.6},
{6.74, 2.0},
{8.19, -0.91},
{9.82, -2.68},
{11.78, -2.88},
{14.14, -1.48},
{16.85, 0.79},
{19.83, 3.11},
{23.03, 4.67},
{26.34, 4.93},
{29.65, 4.13},
{32.8, 2.6},
{35.65, 0.66},
{38.15, -1.49},
{40.38, -3.86},
{42.39, -6.46},
{44.26, -9.28},
{46.08, -12.3},
{48.16, -14.79},
{50.91, -15.67},
{54.19, -15.09},
{57.64, -13.6},
{60.9, -11.79},
{63.76, -9.96},
{66.25, -7.92},
{68.4, -5.43},
{70.02, -2.44},
{70.47, 0.79},
{69.45, 3.97},
{67.34, 6.69},
{64.64, 8.78},
{61.89, 10.65},
{59.65, 12.8},
{58.18, 15.56},
{57.15, 18.92},
{56.16, 22.8},
{55.08, 26.84},
{53.84, 30.61},
{52.41, 33.67},
{50.73, 35.58},
{48.75, 35.92},
{46.48, 34.64},
{43.97, 32.29},
{41.3, 29.44},
{38.54, 26.67},
{35.75, 24.58},
{33.02, 23.7},
{30.36, 24.18},
{27.79, 25.79},
{25.31, 28.25},
{22.92, 31.33},
{20.63, 34.76},
{18.43, 38.3},
{16.32, 41.66},
{14.16, 44.39},
{11.78, 45.97},
{9.02, 45.98},
{5.99, 44.74},
{2.85, 42.79},
{-0.22, 40.67},
{-3.09, 38.76},
{-5.69, 36.83},
{-7.97, 34.52},
{-9.77, 31.64},
{-10.5, 28.41},
{-9.75, 25.16},
{-7.86, 22.33},
{-5.36, 20.23},
{-2.63, 18.41},
{-0.06, 16.22}};

    int left_coords_index{0};
    int right_coords_index{0};
    std::vector<Coord> right_coords{{-7.73, 11.9},
{-4.96, 8.24},
{-2.44, 4.01},
{-0.07, -0.41},
{2.22, -4.63},
{4.5, -8.26},
{6.85, -10.89},
{9.35, -12.16},
{12.06, -11.66},
{15.06, -9.53},
{18.36, -6.82},
{22.01, -4.68},
{25.98, -4.06},
{29.99, -4.92},
{33.66, -6.88},
{36.68, -9.53},
{39.18, -12.56},
{41.55, -15.71},
{44.16, -18.71},
{47.36, -21.3},
{51.04, -23.1},
{54.86, -23.71},
{58.54, -22.88},
{62.04, -20.98},
{65.41, -18.47},
{68.67, -15.85},
{71.79, -13.32},
{74.64, -10.64},
{77.06, -7.53},
{78.81, -3.89},
{79.5, 0.03},
{78.84, 3.94},
{77.06, 7.58},
{74.48, 10.68},
{71.5, 13.35},
{68.51, 15.88},
{65.86, 18.56},
{63.61, 21.57},
{61.62, 25.04},
{59.75, 29.09},
{57.89, 33.66},
{55.95, 38.17},
{53.87, 41.98},
{51.57, 44.44},
{48.97, 44.9},
{46.06, 43.17},
{42.93, 40.11},
{39.71, 36.71},
{36.53, 33.96},
{33.49, 32.87},
{30.64, 33.82},
{27.87, 36.3},
{25.08, 39.73},
{22.16, 43.52},
{19.0, 47.08},
{15.5, 49.85},
{11.74, 51.62},
{7.89, 52.34},
{4.1, 51.97},
{0.48, 50.62},
{-2.99, 48.58},
{-6.33, 46.17},
{-9.57, 43.69},
{-12.62, 41.15},
{-15.36, 38.34},
{-17.63, 35.05},
{-19.13, 31.32},
{-19.45, 27.35},
{-18.43, 23.45},
{-16.4, 20.0},
{-13.73, 17.24},
{-10.73, 14.75},
{-7.73, 11.9}};

    #ifdef DEBUG
    std::cout << "Entering loop" << std::endl;
    #endif

    //Enter refresh loop
    while (1)
    {
        #if defined(VISUALISE_RQT) || defined(TIME_LOG)
            // Recording starting time
            auto start = std::chrono::high_resolution_clock::now();
        #endif

        //MAIN LOOP
        //Collect IMU data - TODO!!
        //Check critical conditions 
        #ifdef TIME_LOG
            if (track->getLapsRaced()<1) function_start = std::chrono::high_resolution_clock::now();
        #endif
        continue_driving = track->carIsInsideTrack();

        #ifdef TIME_LOG
            if (track->getLapsRaced()<1) car_check_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-function_start).count();
        #endif
        //Update track - read-in data if available
        int batch_size {15};
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
       
       if (continue_driving)
       {
            //Check metadata (laps done, goal achievement, etc.)
            track->checkForLap();
            #ifdef TIME_LOG
                if (track->getLapsRaced()<1) function_start = std::chrono::high_resolution_clock::now();
            #endif
            //Process track section
            if (!track->trackIsComplete()) track->processNextSection();
            #ifdef TIME_LOG
                if (track->getLapsRaced()<1) section_process_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-function_start).count();
                if (track->getLapsRaced()<1) function_start = std::chrono::high_resolution_clock::now();
            #endif
            //NEXT MPCC solve
            auto control_inputs = mpcc->solve();  
            #ifdef TIME_LOG
                if (track->getLapsRaced()<1) mpcc_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-function_start).count();
            #endif      
            //Output control_inputs to car - TODO!!
       }
       else
       {
           std::cerr<<"OUTSIDE OF TRACK BOUNDARIES"<<std::endl;
           //Trigger responses to outside of track emergency state - TODO!!
           //break;
       }
       
        #if defined(DEBUG_LOOPS) || defined(DEBUG_SLOW) ||defined(DEBUG_SLOWAFTERLOOPS) ||defined(TIME_LOG)
            loops_completed++;
        #endif
        #ifdef DEBUG_SLOW
            usleep(500000);
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
            if (track->getLapsRaced()<1) 
            {
                auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-start).count();
                time_log->write(ss<<loops_completed<<"           "<<car_check_time<<"            "<<section_process_time<<"        "<<mpcc_time<<"     "<<total_time);
            } else {
                std::cout<<"Lap finished."<<std::endl; break;
            }
        #endif      
    }
   
 } 
