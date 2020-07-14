#pragma once

#include <fstream>
#include <ctime>
#include <iostream>
#include <string>
#include <sstream>


class BoundaryLogger
{
    public:
        BoundaryLogger(std::string file_name, std::string intro_message = "Unspecified", bool reset_log = false);
        ~BoundaryLogger();
        void write(std::ostream &stream, bool final_message = false);
    private:
        std::string addLocalTime();
        std::ofstream fs;
        const static std::string debug_file_loc;
};