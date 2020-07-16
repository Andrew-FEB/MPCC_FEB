#include "visualisation.h"

constexpr int TIME_OUT_VAL = 1;
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
		if (cone_pub.getNumSubscribers()>=1) cone_pub.publish(coneMarkerList);
		if (triangle_pub.getNumSubscribers()>=1) triangle_pub.publish(line_list);
		if (car_pub.getNumSubscribers()>=1) car_pub.publish(carMark);
		if (target_pub.getNumSubscribers()>=1) target_pub.publish(target);
		if (centreCoord_pub.getNumSubscribers()>=1) centreCoord_pub.publish(centreList);
		if (nodeMid_pub.getNumSubscribers()>=1) nodeMid_pub.publish(nodeMidList);
		if (nodeMidConnections_pub.getNumSubscribers()>=1) nodeMidConnections_pub.publish(nodeMidParentsList);
	}
	else
	{
		if (!ros::ok())
		{
			std::cerr<<"ROS reporting not OK in visualise";
		}
		else 
		{
			std::cerr<<"ROS reporting no subscribers in visualise";
		}
	}
}



void Visualisation::showTriangles(const std::vector<triang>& triangles)
{
	if (triangle_pub.getNumSubscribers()<1)
	{
		triangle_pub = n->advertise<visualization_msgs::Marker>("triangle_lines", TIME_OUT_VAL);
	}
	std::cerr<<"Waiting for subscription - triangles."<<std::endl;
	waitForSubscribe(triangle_pub);
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.color.r = 1.0f;
	line_list.color.a = 1.0f;
	line_list.scale.x = 0.05;
	line_list.lifetime = ros::Duration(ROS_DURATION_TIME);
	line_list.header.frame_id = "/tf_bound";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "triangles";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = 0;

	for (const triang &triang : triangles)
	{
		geometry_msgs::Point a, b, c;
		a.x = triang.a.x;
		a.y = triang.a.y;
		b.x = triang.b.x;
		b.y = triang.b.y;
		c.x = triang.c.x;
		c.y = triang.c.y;
		line_list.points.push_back(a);
		line_list.points.push_back(b);
		line_list.points.push_back(b);
		line_list.points.push_back(c);
		line_list.points.push_back(c);
		line_list.points.push_back(a);
	}
	triangle_pub.publish(line_list);
}

void Visualisation::showCones(const std::vector < std::unique_ptr<Cone>> & coneList)
{
	if (cone_pub.getNumSubscribers()<1)
	{
		cone_pub = n->advertise<visualization_msgs::MarkerArray>("cone_markers", TIME_OUT_VAL);
	}
	coneMarkerList.markers.resize(coneList.size());
	std::cerr<<"Waiting for subscription - cones."<<std::endl;
	waitForSubscribe(cone_pub);
	
	for (const std::unique_ptr<Cone>& cone : coneList)
	{
		auto i = &cone - &coneList[0];
		coneMarkerList.markers[i].header.frame_id = "/tf_bound";
		coneMarkerList.markers[i].header.stamp = ros::Time();
		coneMarkerList.markers[i].ns = "cone_images";
		coneMarkerList.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
		coneMarkerList.markers[i].mesh_resource = "file:///home/dm501/MPCC_FEB/Dada/BoundaryEstimation/catkin_ws/src/bound_est/src/resources/meshes/cone.dae";
		coneMarkerList.markers[i].action = visualization_msgs::Marker::ADD;
		coneMarkerList.markers[i].pose.position.z = 0.0;
		coneMarkerList.markers[i].pose.orientation.z = 0.0;
		coneMarkerList.markers[i].pose.orientation.w = 1.0;
		coneMarkerList.markers[i].scale.x = 1.0;
		coneMarkerList.markers[i].scale.y = 1.0;
		coneMarkerList.markers[i].scale.z = 1.0;
		coneMarkerList.markers[i].color.r = 0.0f;
		coneMarkerList.markers[i].color.g = 1.0f;
		coneMarkerList.markers[i].color.b = 0.0f;
		coneMarkerList.markers[i].color.a = 1.0;
		coneMarkerList.markers[i].lifetime = ros::Duration(ROS_DURATION_TIME);
		coneMarkerList.markers[i].pose.position.x = cone->getX();
		coneMarkerList.markers[i].pose.position.y = cone->getY();
		coneMarkerList.markers[i].id = cone->getID();	
	}
	cone_pub.publish(coneMarkerList);
}

void Visualisation::showCar(const coord & car, const double & angle)
{
	if (car_pub.getNumSubscribers()<1)
	{
		car_pub = n->advertise<visualization_msgs::Marker>("car_marker", TIME_OUT_VAL);
	}
	std::cerr<<"Waiting for subscription - car."<<std::endl;
	waitForSubscribe(car_pub);
	carMark.header.frame_id = "/tf_bound";
	carMark.header.stamp = ros::Time();
	carMark.ns = "car_image";
	carMark.type = visualization_msgs::Marker::MESH_RESOURCE;
	carMark.mesh_resource = "file:///home/dm501/MPCC_FEB/Dada/BoundaryEstimation/catkin_ws/src/bound_est/src/resources/meshes/eclipse.stl";
	carMark.action = visualization_msgs::Marker::ADD;
	carMark.pose.position.z = 0.0;
	carMark.pose.orientation.z = tf::createQuaternionMsgFromYaw(angle).z;
	carMark.pose.orientation.w = tf::createQuaternionMsgFromYaw(angle).w;
	carMark.scale.x = 0.5;
	carMark.scale.y = 0.5;
	carMark.scale.z = 0.5;
	carMark.color.r = 0.0f;
	carMark.color.g = 1.0f;
	carMark.color.b = 0.0f;
	carMark.color.a = 1.0;
	carMark.lifetime = ros::Duration(ROS_DURATION_TIME);
	carMark.pose.position.x = car.x;
	carMark.pose.position.y = car.y;
	carMark.id = 0;	
	car_pub.publish(carMark);
}

void Visualisation::showEndPoint(const coord &endPoint)
{
	if (target_pub.getNumSubscribers()<1)
	{
		target_pub = n->advertise<visualization_msgs::Marker>("target_marker", TIME_OUT_VAL);
	}
	std::cerr<<"Waiting for subscription - end points."<<std::endl;
	waitForSubscribe(target_pub);
	
	target.header.frame_id = "/tf_bound";
	target.header.stamp = ros::Time();
	target.ns = "target_image";
	target.type = visualization_msgs::Marker::CYLINDER;
	target.action = visualization_msgs::Marker::ADD;
	target.pose.position.z = 0.0;
	target.pose.orientation.z = 0.0;
	target.pose.orientation.w = 1.0;
	target.scale.x = 0.2;
	target.scale.y = 0.2;
	target.scale.z = 1.0;
	target.color.r = 1.0f;
	target.color.g = 0.0f;
	target.color.b = 0.0f;
	target.color.a = 1.0;
	target.lifetime = ros::Duration(ROS_DURATION_TIME);
	target.pose.position.x = endPoint.x;
	target.pose.position.y = endPoint.y;
	target.id = 0;	
	target_pub.publish(target);
}

void Visualisation::showCentreCoords(const std::vector<coord> &centreCoords)
{
	if (centreCoord_pub.getNumSubscribers()<1)
	{
		centreCoord_pub = n->advertise<visualization_msgs::MarkerArray>("centreCoord_markers", TIME_OUT_VAL);
	}
	centreList.markers.resize(centreCoords.size());
	std::cerr<<"Waiting for subscription - centre coords."<<std::endl;
	waitForSubscribe(centreCoord_pub);

	for (const coord &point : centreCoords)
	{
		auto i = &point - &centreCoords[0];
		centreList.markers[i].header.frame_id = "/tf_bound";
		centreList.markers[i].header.stamp = ros::Time();
		centreList.markers[i].ns = "centreCoord_images";
		centreList.markers[i].type = visualization_msgs::Marker::CUBE;
		centreList.markers[i].action = visualization_msgs::Marker::ADD;
		centreList.markers[i].pose.position.z = 0.0;
		centreList.markers[i].pose.orientation.z = 0.0;
		centreList.markers[i].pose.orientation.w = 1.0;
		centreList.markers[i].scale.x = 0.1;
		centreList.markers[i].scale.y = 0.1;
		centreList.markers[i].scale.z = 1.0;
		centreList.markers[i].color.r = 0.0f;
		centreList.markers[i].color.g = 0.0f;
		centreList.markers[i].color.b = 0.0f;
		centreList.markers[i].color.a = 1.0;
		centreList.markers[i].lifetime = ros::Duration(ROS_DURATION_TIME);
		centreList.markers[i].pose.position.x = point.x;
		centreList.markers[i].pose.position.y = point.y;
		centreList.markers[i].id = i;	
	}
	centreCoord_pub.publish(centreList);
}

void Visualisation::showNodeMids(const std::vector<coord> &midPoints)
{
	if (nodeMid_pub.getNumSubscribers()<1)
	{
		nodeMid_pub = n->advertise<visualization_msgs::MarkerArray>("node_mids", TIME_OUT_VAL);
	}
	nodeMidList.markers.resize(midPoints.size());
	std::cerr<<"Waiting for subscription - node mids."<<std::endl;
	waitForSubscribe(nodeMid_pub);

	for (const coord &point : midPoints)
	{
		auto i = &point - &midPoints[0];
		nodeMidList.markers[i].header.frame_id = "/tf_bound";
		nodeMidList.markers[i].header.stamp = ros::Time();
		nodeMidList.markers[i].ns = "mid_markers";
		nodeMidList.markers[i].type = visualization_msgs::Marker::SPHERE;
		nodeMidList.markers[i].action = visualization_msgs::Marker::ADD;
		nodeMidList.markers[i].pose.position.z = 0.0;
		nodeMidList.markers[i].pose.orientation.z = 0.8;
		nodeMidList.markers[i].pose.orientation.w = 1.0;
		nodeMidList.markers[i].pose.orientation.y = 0.0;
		nodeMidList.markers[i].pose.orientation.x = 0.0;
		nodeMidList.markers[i].scale.x = 0.3;
		nodeMidList.markers[i].scale.y = 0.3;
		nodeMidList.markers[i].scale.z = 0.3;
		nodeMidList.markers[i].color.r = 0.0f;
		nodeMidList.markers[i].color.g = 1.0f;
		nodeMidList.markers[i].color.b = 1.0f;
		nodeMidList.markers[i].color.a = 1.0;
		nodeMidList.markers[i].lifetime = ros::Duration(ROS_DURATION_TIME);
		nodeMidList.markers[i].pose.position.x = point.x;
		nodeMidList.markers[i].pose.position.y = point.y;
		nodeMidList.markers[i].id = i;	
	}
	nodeMid_pub.publish(nodeMidList);
}

void Visualisation::showNodeParentLinks(const std::vector<std::pair<coord, coord>> &connections)
{

	if (nodeMidConnections_pub.getNumSubscribers()<1)
	{
		nodeMidConnections_pub = n->advertise<visualization_msgs::MarkerArray>("nodeParent_lines", TIME_OUT_VAL);
	}
	std::cerr<<"Waiting for subscription - node parent links."<<std::endl;
	waitForSubscribe(nodeMidConnections_pub);
	visualization_msgs::Marker arrow;
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
		nodeMidParentsList.markers.push_back(arrow);
		connection++;
	}
	nodeMidConnections_pub.publish(nodeMidParentsList);
}

