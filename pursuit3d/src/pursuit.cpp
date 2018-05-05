#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "visualization_msgs/Marker.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Quaternion.h"

#include <sstream>

#include "urdf/model.h"
#include "matrices.h"
#include "Geometry3D.h"
#include "Map.h"
#include <iostream>
#include <cmath>
#include <cfloat>
#include <ctime>


struct Agent {
	struct position {
		float x;
		float y;
		float z;
	};
};


class Planner {
	
	std::vector<Agent> p, e;

public:
	Map map;
	Planner();

} planner;

Planner::Planner() {

	// build map
	float th = -15 * M_PI / 180.0;
	mat3 rot({cos(th), -sin(th), 0, sin(th), cos(th), 0, 0, 0, 1});
	OBB *meem = new OBB({-30, 15, 20.4}, {10, 10, 40.8}, rot);
	OBB *chem = new OBB({-5, 20, 14.95}, {8, 20, 29.9}, rot);
	OBB *mub = new OBB({-25, -10, 2.5}, {20, 20, 5}, rot);

	map.AddOBB(meem);
	map.AddOBB(chem);
	map.AddOBB(mub);
	map.Accelerate({0, 0, 20}, 20); // z origin on middle of tallest building

	// create keypoints

	// draw
}

geometry_msgs::Quaternion quat_from_mat3(mat3 mat) {
	tf2::Quaternion quat;
	tf2::Matrix3x3 rot(mat._11, mat._12, mat._13,
					  mat._21, mat._22, mat._23,
					  mat._31, mat._32, mat._33);
	rot.getRotation(quat);

	return(tf2::toMsg(quat));
}

void DrawPlanner(ros::Publisher pub) {
	visualization_msgs::Marker marker;

	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	for(int k = 0; k < planner.map.objects.size(); k++) {
		marker.id = k;
		marker.pose.position.x = planner.map.objects[k]->position.x;
		marker.pose.position.y = planner.map.objects[k]->position.y;
		marker.pose.position.z = planner.map.objects[k]->position.z;
		marker.pose.orientation = quat_from_mat3(planner.map.objects[k]->orientation);
		marker.scale.x = planner.map.objects[k]->size.x;
		marker.scale.y = planner.map.objects[k]->size.y;
		marker.scale.z = planner.map.objects[k]->size.z;

		pub.publish( marker );
	}
}



void radarCallback(const gazebo_msgs::ModelStates) {

	// update agents' pos
	float p1[3] = {1, 2, 3};
	float p2[3] = {1, 2, 3};

	// calculate interception point

	// command agents
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "pursuit");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/gazebo/model_states", 100, radarCallback);
	ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>("marker_map", 1000);


	ros::Rate loop_rate(1);

	int count = 0;
	while(ros::ok()) {
		DrawPlanner(pub_marker);

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