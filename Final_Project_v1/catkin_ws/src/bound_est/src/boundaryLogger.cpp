#include "boundaryLogger.h"

const std::string BoundaryLogger::debug_file_loc = "/home/dm501/MPCC_FEB/Final_Project_v1/catkin_ws/src/bound_est/src/DEBUG_LOG";

BoundaryLogger::BoundaryLogger(std::string file_name, std::string log_type, bool reset_log)
{
    std::string name = debug_file_loc+'/'+file_name+".txt";
    if (reset_log) fs.open(name);
    else fs.open(name, std::ios_base::app);
    if (fs.is_open())
    {
        fs<<"Log message for "<<log_type<<" log data at "<<addLocalTime();
        fs<<"-----------------------------------"<<std::endl;
        fs<<"-----------------------------------"<<std::endl;
    }
    else std::cout<<"Failed to open logging file"<<std::endl;
}

BoundaryLogger::~BoundaryLogger()
{
    if (fs.is_open())
    {
        fs<<"///////////////////////////////////"<<std::endl<<"///////////////////////////////////"<<std::endl<<std::endl;
        fs.close();
    }
}

void BoundaryLogger::write(std::ostream &stream, bool final_message)
{
    if (fs.is_open())
    {
        fs<<stream.rdbuf()<<std::endl;
        if (final_message) fs<<"--------"<<std::endl;
    }
}

std::string BoundaryLogger::addLocalTime()
{
    auto time = std::time(nullptr);
    return std::asctime(std::localtime(&time));
}