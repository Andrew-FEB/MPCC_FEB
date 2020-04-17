#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>
#include <memory>

#include "cone.h"
#include "definitions.h"

constexpr int ROS_DURATION_TIME {1};

class Visualisation
{
public:
	Visualisation(); 
	void configROS();
	void refreshRosOutput();
	void showTriangles(const std::vector<triang> &triangles);
	void showCones(const std::vector < std::unique_ptr<Cone>> &coneList);
	void showCar(const coord & car);
	void showEndPoint(const coord &endPoint);
	void showCentreCoords(const std::vector<coord> &centreCoords);
	void showNodeMids(const std::vector<coord> &midPoints);	
	void showNodeParentLinks(const std::vector<std::pair<coord, coord>> &connections);

private:
	void waitForSubscribe(const ros::Publisher &pub);

	std::unique_ptr<ros::NodeHandle> n;
	ros::Publisher cone_pub;
	ros::Publisher car_pub;
	ros::Publisher triangle_pub;
	ros::Publisher target_pub;
	ros::Publisher centreCoord_pub;
	ros::Publisher nodeMid_pub;
	ros::Publisher nodeMidConnections_pub;

	visualization_msgs::MarkerArray nodeMidList;
	visualization_msgs::MarkerArray nodeMidParentsList;
	visualization_msgs::MarkerArray centreList;
	visualization_msgs::Marker target;
	visualization_msgs::Marker carMark;
	visualization_msgs::MarkerArray coneMarkerList;
	visualization_msgs::Marker line_list;
};

