#include "ros/ros.h"
#include "PursuitWrapper.h"
#include <iostream>

int main(int argc, char **argv) {
	std::srand(3);

	ros::init(argc, argv, "pursuit");

	PursuitWrapper pursuit(Planner(20, 5, 1.0));
	
	while (ros::ok()) {

		pursuit.step();
		ros::spinOnce();
	}

	return 0;
}