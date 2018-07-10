#include "ros/ros.h"
#include "PursuitWrapper.h"
#include <iostream>


int main(int argc, char **argv) {
	std::srand(83);

	ros::init(argc, argv, "test");

	// PursuitWrapper plot(Planner(20, 3, 1.0));
	PursuitWrapper plot;
	
	// plan a path
	SPT* spt = &plot.planner.e[0];
	PathResult result;
	spt->Rewire(spt->root);
	spt->findPath({-50, 30, 20}, result);

	
	plot.BuildPointCloud();
	plot.BuildInterceptCloud();
	plot.DrawConvergence();
	// plot.DrawTree(plot.planner.e[0]);
	// plot.DrawPath(result, 0, {0.55, 0.75, 0.25});

	ros::spin();
	return 0;
}