#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <string>
#include <memory>


#include "cone.h"
#include "definitions.h"

class Visualise
{
public:
	Visualise(); 
	void configROS(std::string process_name = "boundary_est");
	void refreshRosOutput();
	void showTriangles(const std::vector<triang>& triangles);
	void showCones(const std::vector < std::unique_ptr<Cone>> & coneList);
	
	
private:
	ros::NodeHandle n;
	ros::Publisher marker_pub;
};

