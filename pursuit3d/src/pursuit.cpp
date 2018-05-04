#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"

#include <sstream>

#include "urdf/model.h"
#include "Geometry3D.h"
#include "Map.h"
#include <iostream>
#include <cmath>
#include <cfloat>
#include <ctime>


class Planner {
	Map map;

public:
	Planner();

} planner;

Planner::Planner() {

	// build map
	float th = -15 * M_PI / 180.0;
	mat3 rot({cos(th), -sin(th), 0, sin(th), cos(th), 0, 0, 1});
	OBB *meem = new OBB({-50, 30, 20.4}, {10, 10, 40.8}, rot);
	OBB *chem = new OBB({-20, 40, 14.95}, {8, 8, 29.9}, rot);

	map.AddOBB(meem);
	map.AddOBB(chem);
	map.Accelerate({0, 0, 20}, 20); // z origin on middle of tallest building

	// create keypoints
}



void radarCallback(const gazebo_msgs::ModelStates) {

	// update agents' pose

	// calculate interception point

	// command agents
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "pursuit");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/gazebo/model_states", 100, chatterCallback);
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