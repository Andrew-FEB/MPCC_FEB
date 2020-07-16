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
    coord midpoint{(2.5-2.5) / 2, (-2 + -1.5) / 2 };
    coord midpoint_2{(-2.0 + 3.0)/2, (1+1.5)/2};
    double angle = atan2(midpoint_2.y-midpoint.y, midpoint_2.x-midpoint.x)*180/M_PI;
    car->setPosition({midpoint, angle});
    track->addCone(-2.5, -2.0, BoundPos::undefined);    //left
    track->addCone(2.5, -1.5, BoundPos::undefined); //right
    track->addCone(-2.0, 1.0, BoundPos::undefined);   //left
    track->addCone(3.0, 1.5, BoundPos::undefined); //right
    track->addCone(-2.5, 4.0, BoundPos::undefined); //left
    track->addCone(2.6, 3.7, BoundPos::undefined); //right
    track->addCone(-1.0, 6.5, BoundPos::undefined); //left
    track->addCone(5.0, 7.0, BoundPos::undefined); //right
    track->addCone(-0.5, 9.0, BoundPos::undefined); //left
    track->addCone(6.7, 11.3, BoundPos::undefined); //right
    track->addCone(1.0, 13.0, BoundPos::undefined); //left
    track->addCone(8.7, 12.3, BoundPos::undefined); //right
    track->addCone(2.7, 13.6, BoundPos::undefined); //left
    track->addCone(11, 14.5, BoundPos::undefined); //right
    track->addCone(4.9, 14.2, BoundPos::undefined); //left
    track->addCone(12, 16.0, BoundPos::undefined); //right
    track->addCone(6.2, 15.5, BoundPos::undefined); //left
    track->addCone(12.5, 20.3, BoundPos::undefined); //right
    track->addCone(7.0, 19.0, BoundPos::undefined);  //left

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
        *ros::Subscriber cone_sub = cone_reader.subscribe("/map", 1000, chatterCallback);
        *ros::spin();
        */
        //Process track section
        track->processNextSection();
        //TEST
        auto path = track->getReferencePath(1, 10);
        std::cerr<<"ref path size = "<<path.size()<<std::endl;
        //ETEST
        //DADA MPCC
        auto car = track->getCar();
        /*auto control = mpcc->solve(*car, *track);
        //Update car outputs
        car->updateCar(control, time_step);*/
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
        usleep(300000);
        std::cout<<"Loops completed: "<<++loops_completed<<std::endl;

        #endif
        #ifdef DEBUG_LOOPS
        if (++loops_completed>=1) 
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
