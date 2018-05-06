#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "visualization_msgs/Marker.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Quaternion.h"

#include <sstream>

#include "matrices.h"
#include "vectors.h"
#include "Geometry3D.h"
#include "Map.h"
#include "Pursuit.h"
#include <iostream>
#include <cmath>
#include <cfloat>
#include <ctime>

geometry_msgs::Quaternion quat_from_mat3(mat3 mat) {
	tf2::Quaternion quat;
	mat = Transpose(mat);
	tf2::Matrix3x3 rot(mat._11, mat._12, mat._13,
					  mat._21, mat._22, mat._23,
					  mat._31, mat._32, mat._33);
	rot.getRotation(quat);


	return(tf2::toMsg(quat));
}

void DrawGraph(const SPT& spt, ros::Publisher pub) {
	visualization_msgs::Marker marker;

	marker.id = 2000;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;
	marker.points.resize(2);


	for(int i = 0; i < spt.pt.size(); i++) {
		marker.pose.position.x = spt.pt[i].x;
		marker.pose.position.y = spt.pt[i].y;
		marker.pose.position.z = spt.pt[i].z;
		for(int j = i + 1; j < spt.pt.size(); j++) {
			if(spt.graph[i][j] > 0) {
				marker.id++;
				marker.points[1].x = spt.pt[j].x - spt.pt[i].x;
				marker.points[1].y = spt.pt[j].y - spt.pt[i].y;
				marker.points[1].z = spt.pt[j].z - spt.pt[i].z;
				pub.publish( marker );
			}
		}
	}
	// ROS_INFO("%d links created", marker.id - 1000);
}

void DrawObstacles(const Map& map, ros::Publisher pub) {
	visualization_msgs::Marker marker;

	// plot OBB
	marker.id = 0;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 0.5; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	for(int k = 0; k < map.objects.size(); k++) {
		marker.id = k;
		marker.pose.position.x = map.objects[k]->position.x;
		marker.pose.position.y = map.objects[k]->position.y;
		marker.pose.position.z = map.objects[k]->position.z;
		marker.pose.orientation = quat_from_mat3(map.objects[k]->orientation);
		marker.scale.x = map.objects[k]->size.x * 2.0;
		marker.scale.y = map.objects[k]->size.y * 2.0;
		marker.scale.z = map.objects[k]->size.z * 2.0;

		pub.publish( marker );
	}
}

void DrawKeypoints(const SPT& spt, ros::Publisher pub) {
	visualization_msgs::Marker marker;

	// plot keypoints
	marker.id = 100;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;

	for(int k = 0; k < spt.pt.size(); k++) {
		marker.id++;
		marker.pose.position.x = spt.pt[k].x;
		marker.pose.position.y = spt.pt[k].y;
		marker.pose.position.z = spt.pt[k].z;

		pub.publish( marker );
	}
}

void DrawKeypointLabels(const SPT& spt, ros::Publisher pub) {
	visualization_msgs::Marker marker;

	// plot keypoints
	marker.id = 200;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 2.0;
	marker.pose.orientation.w = 1.0;

	for(int k = 0; k < spt.pt.size(); k++) {
		marker.id++;
		marker.pose.position.x = spt.pt[k].x;
		marker.pose.position.y = spt.pt[k].y;
		marker.pose.position.z = spt.pt[k].z + 1;
		marker.text = std::to_string(k);

		pub.publish( marker );
	}
}

void DrawTree(const SPT& spt, ros::Publisher pub) {
	visualization_msgs::Marker marker;

	marker.id = 1000;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;
	marker.points.resize(2);

	for(int i = 0; i < spt.pt.size(); i++) {
		if(spt.parent[i] < 0)
			continue;

		marker.pose.position.x = spt.pt[i].x;
		marker.pose.position.y = spt.pt[i].y;
		marker.pose.position.z = spt.pt[i].z;

		if(spt.parent[i] < spt.pt.size()) {
			marker.points[1].x = spt.pt[spt.parent[i]].x - spt.pt[i].x;
			marker.points[1].y = spt.pt[spt.parent[i]].y - spt.pt[i].y;
			marker.points[1].z = spt.pt[spt.parent[i]].z - spt.pt[i].z;
		}
		else {
			marker.points[1].x = spt.root.x - spt.pt[i].x;
			marker.points[1].y = spt.root.y - spt.pt[i].y;
			marker.points[1].z = spt.root.z - spt.pt[i].z;
		}

		marker.id++;
		pub.publish( marker );
	}
	// ROS_INFO("%d links created", marker.id - 1000);
}

void DrawPath(const PathResult& result, ros::Publisher pub) {
	visualization_msgs::Marker marker;

	marker.id = 700;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.0;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;
	marker.points.resize(2);


	for(int i = 0; i < result.waypts.size() - 1; i++) {
		marker.pose.position.x = result.waypts[i].x;
		marker.pose.position.y = result.waypts[i].y;
		marker.pose.position.z = result.waypts[i].z;

		marker.id++;
		marker.points[1].x = result.waypts[i+1].x - result.waypts[i].x;
		marker.points[1].y = result.waypts[i+1].y - result.waypts[i].y;
		marker.points[1].z = result.waypts[i+1].z - result.waypts[i].z;
		pub.publish( marker );
	}
	// ROS_INFO("%d links created", marker.id - 1000);
}

void radarCallback(const gazebo_msgs::ModelStates) {

	// update agents' pos
	float p1[3] = {1, 2, 3};
	float p2[3] = {1, 2, 3};

	// calculate interception point

	// command agents
}

void BuildCampusMap(Map& map) {
	// build campus map
	float th = -10 * M_PI / 180.0;
	mat3 rot = Transpose(mat3({std::cos(th), -std::sin(th), 0, 
			  std::sin(th), std::cos(th), 0, 
			  0, 0, 1}));
	
	OBB *chem = new OBB({-5, 20, 15}, {4, 10, 15}, rot);
	OBB *mub = new OBB({-30, -15, 2.5}, {10, 10, 2.5}, rot);
	OBB *meem = new OBB({-30, 15, 20}, {5, 5, 20}, rot);

	map.AddOBB(meem);
	map.AddOBB(chem);
	map.AddOBB(mub);
	// map.Accelerate({0, 0, 50}, 50); // z origin on middle of tallest building
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "pursuit");
	ros::NodeHandle n;

	Map map;
	BuildCampusMap(map);

	Planner planner(&map);
	planner.AddPursuer({10, 10, 2});
	planner.AddEvader({-50, 20, 2});
	planner.AddGoal({10, 30, 1});

	ros::Subscriber sub = n.subscribe("/gazebo/model_states", 100, radarCallback);
	ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>("marker_map", 1000);
	
	InterceptionResult itcp;
	clock_t begin = clock();
	bool found = planner.EvaluatePoint({-20, 0, 10}, itcp);
	clock_t end = clock();

	ROS_INFO("Interception found? %d\nCost: %f\nConstraint: %f\nCost grad: ", found, itcp.cost, itcp.constraint);
	std::cout << itcp.costd << "\nConstraint grad: " << itcp.constraintd << "\n";

	ROS_INFO("Elapsed time: %.5f us\n",
		1e6 * (double(end - begin) / CLOCKS_PER_SEC) / 1);
	
	ros::Rate loop_rate(10);

	int count = 0;
	while(ros::ok()) {

		DrawObstacles(map, pub_marker);
		DrawKeypoints(planner.p[0], pub_marker);
		DrawKeypointLabels(planner.p[0], pub_marker);
		// DrawGraph(ptree, pub_marker);
		DrawTree(planner.p[0], pub_marker);
		DrawPath(itcp.ppath, pub_marker);
		
		std_msgs::String msg;
		std::stringstream ss;
		ss << "Hello World! " << count;
		msg.data = ss.str();

		// ROS_INFO("%s", msg.data.c_str());
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
	
}