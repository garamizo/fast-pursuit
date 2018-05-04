#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "urdf/model.h"
#include "Geometry3D.h"
#include "Map.h"
#include <iostream>
#include <cmath>
#include <cfloat>
#include <ctime>

int main(int argc, char **argv) {
	ros::init(argc, argv, "pursuit");
	if (argc != 2) {
		ROS_ERROR("Need a urdf file as argument");
		return -1;
	}
	std::string urdf_file = argv[1];

	urdf::Model model;
	if(!model.initFile(urdf_file)) {
		ROS_ERROR("Failed to parse urdf file");
		return -1;
	}
	ROS_INFO("Successfully parsed urdf file");

	boost::shared_ptr<const urdf::Link> link = model.getLink("link1");
	std::cout << link->collision->origin.position.x << std::endl;

	std::vector<boost::shared_ptr<urdf::Link>> links;
	model.getLinks(links);
	std::cout << links[1]->collision->origin.position.x << std::endl;


	return 0;

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while(ros::ok()) {
		std_msgs::String msg;
		std::stringstream ss;
		ss << "Hello World! " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}