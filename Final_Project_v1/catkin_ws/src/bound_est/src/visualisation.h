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
	void showNewCones(const std::vector < std::unique_ptr<Cone>> &coneList);
	void showOldCones(const std::vector < std::unique_ptr<Cone>> &coneList);
	void showFramedCones(const std::vector < std::unique_ptr<Cone>> &coneList);
	void showCar(const Pos &pos);
	void showCarDirection(const Pos &pos);
	void showEndPoint(const coord &endPoint);
	void showCentreCoords(const std::vector<coord> &centreCoords);
	void showNodeMids(const std::vector<coord> &midPoints);	
	void showNodeParentLinks(const std::vector<std::pair<coord, coord>> &connections);
	void showViablePaths(const std::vector<std::vector<coord>> &paths, bool refresh = false);
	void showLeftCones(const std::vector <const Cone *> &coneList);
	void showRightCones(const std::vector <const Cone *> &coneList);
	void showReferencePath(const std::vector<MPC_targets> &reference_path);
	void showCarBoundaryPoints(const Rect &car_rect, const bool &outside_track);

private:
	void waitForSubscribe(const ros::Publisher &pub);

	std::unique_ptr<ros::NodeHandle> n;
	ros::Publisher new_cone_pub;
	ros::Publisher framed_cone_pub;
	ros::Publisher old_cone_pub;
	ros::Publisher car_pub;
	ros::Publisher car_direction_pub;
	ros::Publisher triangle_pub;
	ros::Publisher target_pub;
	ros::Publisher centre_coord_pub;
	ros::Publisher node_mid_pub;
	ros::Publisher node_mid_connections_pub;
	ros::Publisher paths_pub;
	ros::Publisher left_cone_pub;
	ros::Publisher right_cone_pub;
	ros::Publisher reference_point_pub;
	ros::Publisher boundary_slope_pub;
	ros::Publisher boundary_point_pub;
	ros::Publisher reference_to_boundary_pub;
	ros::Publisher car_boundary_pub;

	visualization_msgs::MarkerArray node_mid_markers;
	visualization_msgs::MarkerArray node_mid_parent_markers;
	visualization_msgs::MarkerArray path_markers;
	visualization_msgs::MarkerArray centre_markers;
	visualization_msgs::Marker target_marker;
	visualization_msgs::Marker car_marker;
	visualization_msgs::Marker car_direction_marker;
	visualization_msgs::MarkerArray new_cone_markers;
	visualization_msgs::MarkerArray framed_cone_markers;
	visualization_msgs::MarkerArray old_cone_markers;
	visualization_msgs::Marker triangle_line_markers;
	visualization_msgs::MarkerArray left_cone_markers;
	visualization_msgs::MarkerArray right_cone_markers;
	visualization_msgs::MarkerArray reference_point_markers;
	visualization_msgs::Marker boundary_slope_markers;
	visualization_msgs::MarkerArray boundary_point_markers;
	visualization_msgs::Marker reference_to_boundary_markers;
	visualization_msgs::MarkerArray car_boundary_markers;
};

