#include "visualisation.h"

constexpr int TIME_OUT_VAL = 1;
constexpr int NEW_CONE_TIME_OUT_VAL = 3;
constexpr int RATE_VAL = 1;

Visualisation::Visualisation()
{
	configROS();
}

inline void Visualisation::waitForSubscribe(const ros::Publisher &pub)
{
	while (pub.getNumSubscribers() < 1)
    {
    if (!ros::ok())
      {
        return;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
}

void Visualisation::configROS()
{
	n = std::make_unique<ros::NodeHandle>();
	ros::Rate r(RATE_VAL);
}

void Visualisation::refreshRosOutput()
{
	if (ros::ok())
	{
		if (new_cone_pub.getNumSubscribers()>=1) new_cone_pub.publish(new_cone_markers);
		if (framed_cone_pub.getNumSubscribers()>=1) new_cone_pub.publish(framed_cone_markers);
		if (old_cone_pub.getNumSubscribers()>=1) old_cone_pub.publish(old_cone_markers);
		if (triangle_pub.getNumSubscribers()>=1) triangle_pub.publish(triangle_line_markers);
		if (car_pub.getNumSubscribers()>=1) car_pub.publish(car_marker);
		if (car_direction_pub.getNumSubscribers()>=1) car_direction_pub.publish(car_direction_marker);
		if (target_pub.getNumSubscribers()>=1) target_pub.publish(target_marker);
		if (centre_coord_pub.getNumSubscribers()>=1) centre_coord_pub.publish(centre_markers);
		if (node_mid_pub.getNumSubscribers()>=1) node_mid_pub.publish(node_mid_markers);
		if (node_mid_connections_pub.getNumSubscribers()>=1) node_mid_connections_pub.publish(node_mid_parent_markers);
		if (paths_pub.getNumSubscribers()>=1) paths_pub.publish(path_markers);
		if (right_cone_pub.getNumSubscribers()>=1) right_cone_pub.publish(right_cone_markers);
		if (left_cone_pub.getNumSubscribers()>=1) left_cone_pub.publish(left_cone_markers);
	}
	else
	{
		if (!ros::ok())
		{
			std::cerr<<"ROS reporting not OK in Visualisation";
		}
		else 
		{
			std::cerr<<"ROS reporting no subscribers in Visualisation";
		}
	}
}



void Visualisation::showTriangles(const std::vector<triang>& triangles)
{
	if (triangle_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - triangles."<<std::endl;
		triangle_pub = n->advertise<visualization_msgs::Marker>("triangle_lines", TIME_OUT_VAL);
		waitForSubscribe(triangle_pub);
	}
	if (!triangle_line_markers.points.empty())
	{
		triangle_line_markers.points.clear();
	}
	triangle_line_markers.type = visualization_msgs::Marker::LINE_LIST;
	triangle_line_markers.color.r = 1.0f;
	triangle_line_markers.color.a = 1.0f;
	triangle_line_markers.scale.x = 0.05;
	triangle_line_markers.lifetime = ros::Duration(ROS_DURATION_TIME);
	triangle_line_markers.header.frame_id = "/tf_bound";
	triangle_line_markers.header.stamp = ros::Time::now();
	triangle_line_markers.ns = "triangles";
	triangle_line_markers.action = visualization_msgs::Marker::ADD;
	triangle_line_markers.pose.orientation.w = 1.0;
	triangle_line_markers.id = 0;

	for (const triang &triang : triangles)
	{
		geometry_msgs::Point a, b, c;
		a.x = triang.a.x;
		a.y = triang.a.y;
		b.x = triang.b.x;
		b.y = triang.b.y;
		c.x = triang.c.x;
		c.y = triang.c.y;
		triangle_line_markers.points.push_back(a);
		triangle_line_markers.points.push_back(b);
		triangle_line_markers.points.push_back(b);
		triangle_line_markers.points.push_back(c);
		triangle_line_markers.points.push_back(c);
		triangle_line_markers.points.push_back(a);
	}
	triangle_pub.publish(triangle_line_markers);	
	std::cerr<<"Visualisation connection completed - triangles."<<std::endl;
}

void Visualisation::showNewCones(const std::vector<std::unique_ptr<Cone>> &coneList)
{
	if (new_cone_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - new cones."<<std::endl;
		new_cone_pub = n->advertise<visualization_msgs::MarkerArray>("new_cone_markers", NEW_CONE_TIME_OUT_VAL);
		waitForSubscribe(new_cone_pub);
	}
	if (!new_cone_markers.markers.empty())
	{
		new_cone_markers.markers.clear();
	}
	new_cone_markers.markers.resize(coneList.size());
	for (const std::unique_ptr<Cone>& cone : coneList)
	{
		auto i = &cone - &coneList[0];
		new_cone_markers.markers[i].header.frame_id = "/tf_bound";
		new_cone_markers.markers[i].header.stamp = ros::Time();
		new_cone_markers.markers[i].ns = "new_cones";
		new_cone_markers.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
		new_cone_markers.markers[i].mesh_resource = "file:///home/dm501/catkin_ws/src/bound_est/src/resources/meshes/cone.dae";
		new_cone_markers.markers[i].action = visualization_msgs::Marker::ADD;
		new_cone_markers.markers[i].pose.position.z = 0.0;
		new_cone_markers.markers[i].pose.orientation.z = 0.0;
		new_cone_markers.markers[i].pose.orientation.w = 1.0;
		new_cone_markers.markers[i].scale.x = 1.5;
		new_cone_markers.markers[i].scale.y = 1.5;
		new_cone_markers.markers[i].scale.z = 1.5;
		new_cone_markers.markers[i].color.r = 1.0f;
		new_cone_markers.markers[i].color.g = 1.0f;
		new_cone_markers.markers[i].color.b = 1.0f;
		new_cone_markers.markers[i].color.a = 1.0;
		new_cone_markers.markers[i].lifetime = ros::Duration(ROS_DURATION_TIME);
		new_cone_markers.markers[i].pose.position.x = cone->getX();
		new_cone_markers.markers[i].pose.position.y = cone->getY();
		new_cone_markers.markers[i].id = cone->getID();	
	}
	new_cone_pub.publish(new_cone_markers);
	std::cerr<<"Visualisation connection completed - new cones."<<std::endl;
}

void Visualisation::showFramedCones(const std::vector<std::unique_ptr<Cone>> &coneList)
{
	if (framed_cone_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - framed cones."<<std::endl;
		framed_cone_pub = n->advertise<visualization_msgs::MarkerArray>("framed_cone_markers", TIME_OUT_VAL);
		waitForSubscribe(framed_cone_pub);
	}
	if (!framed_cone_markers.markers.empty())
	{
		framed_cone_markers.markers.clear();
	}
	framed_cone_markers.markers.resize(coneList.size());
	for (const std::unique_ptr<Cone>& cone : coneList)
	{
		auto i = &cone - &coneList[0];
		framed_cone_markers.markers[i].header.frame_id = "/tf_bound";
		framed_cone_markers.markers[i].header.stamp = ros::Time();
		framed_cone_markers.markers[i].ns = "new_cones";
		framed_cone_markers.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
		framed_cone_markers.markers[i].mesh_resource = "file:///home/dm501/catkin_ws/src/bound_est/src/resources/meshes/cone.dae";
		framed_cone_markers.markers[i].action = visualization_msgs::Marker::ADD;
		framed_cone_markers.markers[i].pose.position.z = 0.0;
		framed_cone_markers.markers[i].pose.orientation.z = 0.0;
		framed_cone_markers.markers[i].pose.orientation.w = 1.0;
		framed_cone_markers.markers[i].scale.x = 3.0;
		framed_cone_markers.markers[i].scale.y = 3.0;
		framed_cone_markers.markers[i].scale.z = 3.0;
		framed_cone_markers.markers[i].color.r = 1.0f;
		framed_cone_markers.markers[i].color.g = 1.0f;
		framed_cone_markers.markers[i].color.b = 1.0f;
		framed_cone_markers.markers[i].color.a = 1.0;
		framed_cone_markers.markers[i].lifetime = ros::Duration(ROS_DURATION_TIME);
		framed_cone_markers.markers[i].pose.position.x = cone->getX();
		framed_cone_markers.markers[i].pose.position.y = cone->getY();
		framed_cone_markers.markers[i].id = cone->getID();	
	}
	framed_cone_pub.publish(framed_cone_markers);
	std::cerr<<"Visualisation connection completed - framed cones."<<std::endl;
}

void Visualisation::showOldCones(const std::vector<std::unique_ptr<Cone>> &coneList)
{
	if (old_cone_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - old cones."<<std::endl;
		old_cone_pub = n->advertise<visualization_msgs::MarkerArray>("old_cone_markers", TIME_OUT_VAL);
		waitForSubscribe(old_cone_pub);
	}
	if (!old_cone_markers.markers.empty())
	{
		old_cone_markers.markers.clear();
	}
	old_cone_markers.markers.resize(coneList.size());
	for (const std::unique_ptr<Cone>& cone : coneList)
	{
		auto i = &cone - &coneList[0];
		old_cone_markers.markers[i].header.frame_id = "/tf_bound";
		old_cone_markers.markers[i].header.stamp = ros::Time();
		old_cone_markers.markers[i].ns = "old_cones";
		old_cone_markers.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
		old_cone_markers.markers[i].mesh_resource = "file:///home/dm501/catkin_ws/src/bound_est/src/resources/meshes/cone.dae";
		old_cone_markers.markers[i].action = visualization_msgs::Marker::ADD;
		old_cone_markers.markers[i].pose.position.z = 0.0;
		old_cone_markers.markers[i].pose.orientation.z = 0.0;
		old_cone_markers.markers[i].pose.orientation.w = 1.0;
		old_cone_markers.markers[i].scale.x = 0.7;
		old_cone_markers.markers[i].scale.y = 0.7;
		old_cone_markers.markers[i].scale.z = 0.7;
		old_cone_markers.markers[i].color.r = 0.0f;
		old_cone_markers.markers[i].color.g = 1.0f;
		old_cone_markers.markers[i].color.b = 0.0f;
		old_cone_markers.markers[i].color.a = 0.7;
		old_cone_markers.markers[i].lifetime = ros::Duration(ROS_DURATION_TIME);
		old_cone_markers.markers[i].pose.position.x = cone->getX();
		old_cone_markers.markers[i].pose.position.y = cone->getY();
		old_cone_markers.markers[i].id = cone->getID();	
	}
	old_cone_pub.publish(old_cone_markers);
	std::cerr<<"Visualisation connection completed - old cones."<<std::endl;
}

void Visualisation::showCar(const Pos &pos)
{
	if (car_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - car."<<std::endl;
		car_pub = n->advertise<visualization_msgs::Marker>("car_marker", TIME_OUT_VAL);
		waitForSubscribe(car_pub);
	}
	car_marker.header.frame_id = "/tf_bound";
	car_marker.header.stamp = ros::Time();
	car_marker.ns = "car_image";
	car_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	car_marker.mesh_resource = "file:///home/dm501/catkin_ws/src/bound_est/src/resources/meshes/eclipse.stl";
	car_marker.action = visualization_msgs::Marker::ADD;
	car_marker.pose.position.z = 0.0;
	car_marker.pose.orientation.z = 0.0;
	car_marker.pose.orientation.w = 1.0;
	car_marker.scale.x = 0.5;
	car_marker.scale.y = 0.5;
	car_marker.scale.z = 0.5;
	car_marker.color.r = 0.0f;
	car_marker.color.g = 1.0f;
	car_marker.color.b = 0.0f;
	car_marker.color.a = 1.0;
	car_marker.lifetime = ros::Duration(ROS_DURATION_TIME);
	car_marker.pose.position.x = pos.p.x;
	car_marker.pose.position.y = pos.p.y;
	car_marker.id = 0;	
	car_pub.publish(car_marker);
	std::cerr<<"Visualisation connection completed - car."<<std::endl;
}

void Visualisation::showCarDirection(const Pos &pos)
{
	if (car_direction_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - car direction."<<std::endl;
		car_direction_pub = n->advertise<visualization_msgs::Marker>("car_direction_marker", TIME_OUT_VAL);
		waitForSubscribe(car_direction_pub);
	}
	
	car_direction_marker.header.frame_id = "/tf_bound";
	car_direction_marker.header.stamp = ros::Time();
	car_direction_marker.ns = "car_direction_image";
	car_direction_marker.type = visualization_msgs::Marker::LINE_LIST;
	car_direction_marker.action = visualization_msgs::Marker::ADD;
	car_direction_marker.pose.position.z = 0.0;
	car_direction_marker.pose.orientation.z = 0.0;
	car_direction_marker.pose.orientation.w = 1.0;
	car_direction_marker.scale.x = 0.05;
	car_direction_marker.color.r = 1.0f;
	car_direction_marker.color.g = 0.0f;
	car_direction_marker.color.b = 1.0f;
	car_direction_marker.color.a = 1.0;
	car_direction_marker.lifetime = ros::Duration(ROS_DURATION_TIME);
	car_direction_marker.id = 0;	
	geometry_msgs::Point car_pos, car_project_pos;
	car_pos.x = pos.p.x;
	car_pos.y = pos.p.y;
	car_direction_marker.points.push_back(car_pos);
	car_project_pos.x = cos(pos.phi*M_PI/180)*3+pos.p.x;
	car_project_pos.y = sin(pos.phi*M_PI/180)*3+pos.p.y;
	car_direction_marker.points.push_back(car_project_pos);
	car_direction_pub.publish(car_direction_marker);
	std::cerr<<"Visualisation connection completed - car direction."<<std::endl;
}

void Visualisation::showEndPoint(const coord &endPoint)
{
	if (target_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - end points."<<std::endl;
		target_pub = n->advertise<visualization_msgs::Marker>("target_marker", TIME_OUT_VAL);
		waitForSubscribe(target_pub);
	}
	
	target_marker.header.frame_id = "/tf_bound";
	target_marker.header.stamp = ros::Time();
	target_marker.ns = "target_image";
	target_marker.type = visualization_msgs::Marker::CYLINDER;
	target_marker.action = visualization_msgs::Marker::ADD;
	target_marker.pose.position.z = 0.0;
	target_marker.pose.orientation.z = 0.0;
	target_marker.pose.orientation.w = 1.0;
	target_marker.scale.x = 0.2;
	target_marker.scale.y = 0.2;
	target_marker.scale.z = 1.0;
	target_marker.color.r = 1.0f;
	target_marker.color.g = 0.0f;
	target_marker.color.b = 0.0f;
	target_marker.color.a = 1.0;
	target_marker.lifetime = ros::Duration(ROS_DURATION_TIME);
	target_marker.pose.position.x = endPoint.x;
	target_marker.pose.position.y = endPoint.y;
	target_marker.id = 0;	
	target_pub.publish(target_marker);
	std::cerr<<"Visualisation connection completed - end point."<<std::endl;
}

void Visualisation::showCentreCoords(const std::vector<coord> &centreCoords)
{
	if (centre_coord_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - centre coords."<<std::endl;
		centre_coord_pub = n->advertise<visualization_msgs::MarkerArray>("centreCoord_markers", TIME_OUT_VAL);
		waitForSubscribe(centre_coord_pub);
	}
	if (!centre_markers.markers.empty())
	{
		centre_markers.markers.clear();
	}
	centre_markers.markers.resize(centreCoords.size());
	
	for (const coord &point : centreCoords)
	{
		auto i = &point - &centreCoords[0];
		centre_markers.markers[i].header.frame_id = "/tf_bound";
		centre_markers.markers[i].header.stamp = ros::Time();
		centre_markers.markers[i].ns = "centreCoord_images";
		centre_markers.markers[i].type = visualization_msgs::Marker::CUBE;
		centre_markers.markers[i].action = visualization_msgs::Marker::ADD;
		centre_markers.markers[i].pose.position.z = 0.0;
		centre_markers.markers[i].pose.orientation.z = 0.0;
		centre_markers.markers[i].pose.orientation.w = 1.0;
		centre_markers.markers[i].scale.x = 0.1;
		centre_markers.markers[i].scale.y = 0.1;
		centre_markers.markers[i].scale.z = 1.0;
		centre_markers.markers[i].color.r = 1.0f;
		centre_markers.markers[i].color.g = 1.0f;
		centre_markers.markers[i].color.b = 0.0f;
		centre_markers.markers[i].color.a = 1.0;
		centre_markers.markers[i].lifetime = ros::Duration(ROS_DURATION_TIME);
		centre_markers.markers[i].pose.position.x = point.x;
		centre_markers.markers[i].pose.position.y = point.y;
		centre_markers.markers[i].id = i;	
	}
	centre_coord_pub.publish(centre_markers);
	std::cerr<<"Visualisation connection completed - centre coords."<<std::endl;
}

void Visualisation::showNodeMids(const std::vector<coord> &midPoints)
{
	if (node_mid_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - node mids."<<std::endl;
		node_mid_pub = n->advertise<visualization_msgs::MarkerArray>("node_mids", TIME_OUT_VAL);
		waitForSubscribe(node_mid_pub);
	}
	if (!node_mid_markers.markers.empty())
	{
		node_mid_markers.markers.clear();
	}
	node_mid_markers.markers.resize(midPoints.size());

	for (const coord &point : midPoints)
	{
		auto i = &point - &midPoints[0];
		node_mid_markers.markers[i].header.frame_id = "/tf_bound";
		node_mid_markers.markers[i].header.stamp = ros::Time();
		node_mid_markers.markers[i].ns = "mid_markers";
		node_mid_markers.markers[i].type = visualization_msgs::Marker::SPHERE;
		node_mid_markers.markers[i].action = visualization_msgs::Marker::ADD;
		node_mid_markers.markers[i].pose.position.z = 0.0;
		node_mid_markers.markers[i].pose.orientation.z = 0.8;
		node_mid_markers.markers[i].pose.orientation.w = 1.0;
		node_mid_markers.markers[i].pose.orientation.y = 0.0;
		node_mid_markers.markers[i].pose.orientation.x = 0.0;
		node_mid_markers.markers[i].scale.x = 0.3;
		node_mid_markers.markers[i].scale.y = 0.3;
		node_mid_markers.markers[i].scale.z = 0.3;
		node_mid_markers.markers[i].color.r = 0.0f;
		node_mid_markers.markers[i].color.g = 1.0f;
		node_mid_markers.markers[i].color.b = 1.0f;
		node_mid_markers.markers[i].color.a = 1.0;
		node_mid_markers.markers[i].lifetime = ros::Duration(ROS_DURATION_TIME);
		node_mid_markers.markers[i].pose.position.x = point.x;
		node_mid_markers.markers[i].pose.position.y = point.y;
		node_mid_markers.markers[i].id = i;	
	}
	node_mid_pub.publish(node_mid_markers);
	std::cerr<<"Visualisation connection completed - node mids."<<std::endl;
}

void Visualisation::showNodeParentLinks(const std::vector<std::pair<coord, coord>> &connections)
{

	if (node_mid_connections_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - node parent links."<<std::endl;
		node_mid_connections_pub = n->advertise<visualization_msgs::MarkerArray>("nodeParent_lines", TIME_OUT_VAL);
		waitForSubscribe(node_mid_connections_pub);
	}
	if (!node_mid_parent_markers.markers.empty())
	{
		node_mid_parent_markers.markers.clear();
	}
	auto connection = connections.begin();
	int index = 0;
	geometry_msgs::Point child, parent;
	while (connection != connections.end())
	{
		visualization_msgs::Marker arrow;
		arrow.header.frame_id = "/tf_bound";
		arrow.header.stamp = ros::Time();
		arrow.ns = "mid_connections";
		arrow.type = visualization_msgs::Marker::ARROW;
		arrow.action = visualization_msgs::Marker::ADD;
		arrow.id = index++;	
		arrow.pose.orientation.z = 0.0;
		arrow.pose.orientation.w = 1.0;
		arrow.pose.orientation.y = 0.0;
		arrow.pose.orientation.x = 0.0;
		child.x = connection->first.x;
		child.y = connection->first.y;
		parent.x = connection->second.x;
		parent.y = connection->second.y;
		arrow.points.push_back(child); 
		arrow.points.push_back(parent); 
		
		arrow.scale.x = 0.05;	//shaft diameter
		arrow.scale.y = 0.2;	//head diameter
		arrow.scale.z = 0.2;	//head length
	
		arrow.color.r = 0.0f;
		arrow.color.g = 0.5f;
		arrow.color.b = 1.0f;
		arrow.color.a = 0.7;

		arrow.lifetime = ros::Duration(ROS_DURATION_TIME);
		node_mid_parent_markers.markers.push_back(arrow);
		connection++;
	}
	node_mid_connections_pub.publish(node_mid_parent_markers);
	std::cerr<<"Visualisation connection completed - node parent links."<<std::endl;
}

void Visualisation::showViablePaths(const std::vector<std::vector<coord>> &paths, bool refresh)
{
	if (paths_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - paths."<<std::endl;
		paths_pub = n->advertise<visualization_msgs::MarkerArray>("path_lines", TIME_OUT_VAL);
		waitForSubscribe(paths_pub);
	}
	int index;
	if (refresh)
	{
		path_markers.markers.clear();
		index = 0;
	}
	else
	{
		if (!path_markers.markers.empty())
		{
			index = path_markers.markers.back().id++;
		}
	}
	geometry_msgs::Point child, parent;
	coord previous_point = {0,0};
	for (auto path : paths)
	{
		bool first{true};

		
		for (auto link : path)
		{
			if (!first)
			{
				visualization_msgs::Marker line;
				line.header.frame_id = "/tf_bound";
				line.header.stamp = ros::Time();
				line.ns = "path_connections";
				line.type = visualization_msgs::Marker::ARROW;
				line.action = visualization_msgs::Marker::ADD;
				line.id = index++;	
				line.pose.orientation.z = path.size()%1;
				line.pose.orientation.w = 1.0;
				line.pose.orientation.y = 0.0;
				line.pose.orientation.x = 0.0;
				child.x = link.x;
				child.y = link.y;
				parent.x = previous_point.x;
				parent.y = previous_point.y;
				line.points.push_back(child); 
				line.points.push_back(parent); 
				
				line.scale.x = 0.2;
				line.scale.y = 0.2;	
				line.scale.z = 0.2;	
			
				line.color.r = 0.0f;
				line.color.g = 1.0f;
				line.color.b = 1.0f;
				line.color.a = 0.7;

				line.lifetime = ros::Duration(ROS_DURATION_TIME);
				path_markers.markers.push_back(line);
			}
			previous_point = {link.x, link.y};
			first = false;
		}
	}
	paths_pub.publish(path_markers);
	std::cerr<<"Visualisation connection completed - paths."<<std::endl;
}

void Visualisation::showLeftCones(const std::vector <const Cone *> &coneList)
{
	if (left_cone_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - left cones."<<std::endl;
		left_cone_pub = n->advertise<visualization_msgs::MarkerArray>("left_cone_markers", TIME_OUT_VAL);
		waitForSubscribe(left_cone_pub);
	}
	if (!left_cone_markers.markers.empty())
	{
		left_cone_markers.markers.clear();
	}
	left_cone_markers.markers.resize(coneList.size());
	int i = 0;
	for (std::vector<const Cone *>::const_iterator it = coneList.begin(); it!=coneList.end(); it++)
	{
		left_cone_markers.markers[i].header.frame_id = "/tf_bound";
		left_cone_markers.markers[i].header.stamp = ros::Time();
		left_cone_markers.markers[i].ns = "left_cones";
		left_cone_markers.markers[i].type = visualization_msgs::Marker::CUBE;
		left_cone_markers.markers[i].action = visualization_msgs::Marker::ADD;
		left_cone_markers.markers[i].pose.position.z = 2.0;
		left_cone_markers.markers[i].pose.orientation.z = 0.0;
		left_cone_markers.markers[i].pose.orientation.w = 1.0;
		left_cone_markers.markers[i].scale.x = 0.2;
		left_cone_markers.markers[i].scale.y = 0.2;
		left_cone_markers.markers[i].scale.z = 0.2;
		left_cone_markers.markers[i].color.r = 0.0f;
		left_cone_markers.markers[i].color.g = 1.0f;
		left_cone_markers.markers[i].color.b = 1.0f;
		left_cone_markers.markers[i].color.a = 1.0;
		left_cone_markers.markers[i].lifetime = ros::Duration(ROS_DURATION_TIME);
		left_cone_markers.markers[i].pose.position.x = (*it)->getX();
		left_cone_markers.markers[i].pose.position.y = (*it)->getY();
		left_cone_markers.markers[i].id = (*it)->getID();	
		i++;
	}
	left_cone_pub.publish(left_cone_markers);
	std::cerr<<"Visualisation connection completed - left cones."<<std::endl;
}

void Visualisation::showRightCones(const std::vector <const Cone *> & coneList)
{
	if (right_cone_pub.getNumSubscribers()<1)
	{
		std::cerr<<"Waiting for subscription - right cones."<<std::endl;
		right_cone_pub = n->advertise<visualization_msgs::MarkerArray>("right_cone_markers", TIME_OUT_VAL);
		waitForSubscribe(right_cone_pub);
	}
	if (!right_cone_markers.markers.empty())
	{
		right_cone_markers.markers.clear();
	}
	right_cone_markers.markers.resize(coneList.size());
	int i = 0;
	for (std::vector<const Cone *>::const_iterator it = coneList.begin(); it!=coneList.end(); it++)
	{
		right_cone_markers.markers[i].header.frame_id = "/tf_bound";
		right_cone_markers.markers[i].header.stamp = ros::Time();
		right_cone_markers.markers[i].ns = "right_cones";
		right_cone_markers.markers[i].type = visualization_msgs::Marker::CUBE;
		right_cone_markers.markers[i].action = visualization_msgs::Marker::ADD;
		right_cone_markers.markers[i].pose.position.z = 2.0;
		right_cone_markers.markers[i].pose.orientation.z = 0.0;
		right_cone_markers.markers[i].pose.orientation.w = 1.0;
		right_cone_markers.markers[i].scale.x = 0.2;
		right_cone_markers.markers[i].scale.y = 0.2;
		right_cone_markers.markers[i].scale.z = 0.2;
		right_cone_markers.markers[i].color.r = 1.0f;
		right_cone_markers.markers[i].color.g = 1.0f;
		right_cone_markers.markers[i].color.b = 0.0f;
		right_cone_markers.markers[i].color.a = 1.0;
		right_cone_markers.markers[i].lifetime = ros::Duration(ROS_DURATION_TIME);
		right_cone_markers.markers[i].pose.position.x = (*it)->getX();
		right_cone_markers.markers[i].pose.position.y = (*it)->getY();
		right_cone_markers.markers[i].id = (*it)->getID();	
		i++;
	}
	right_cone_pub.publish(right_cone_markers);
	std::cerr<<"Visualisation connection completed - right cones."<<std::endl;
}

