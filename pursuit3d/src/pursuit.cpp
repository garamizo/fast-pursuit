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
#include "vectors.h"
#include "Geometry3D.h"
#include "Map.h"
#include <iostream>
#include <cmath>
#include <cfloat>
#include <ctime>


bool SimplePointInMap(const Map* map, const Point pt) {
	for(int k = 0; k < map->objects.size(); k++)
		if(ContainsPoint(*(map->objects[k]), pt))
			return(true);
	return(false);
}


struct Agent {
	struct position {
		float x;
		float y;
		float z;
	};
};


class Planner {
	
	std::vector<Agent> p, e;
	const float edge_resolution = 5.0f;
	const float inflate = 1.0001f;

public:
	std::vector<Point> pt;
	Map map;
	Planner();

} planner;

Planner::Planner() {

	// build map
	float th = -10 * M_PI / 180.0;
	mat3 rot = Transpose(mat3({std::cos(th), -std::sin(th), 0, 
			  std::sin(th), std::cos(th), 0, 
			  0, 0, 1}));

	OBB *meem = new OBB({-30, 15, 20}, {5, 5, 20}, rot);
	OBB *chem = new OBB({-5, 20, 15}, {4, 10, 15}, rot);
	OBB *mub = new OBB({-30, -15, 2.5}, {10, 10, 2.5}, rot);

	map.AddOBB(meem);
	map.AddOBB(chem);
	map.AddOBB(mub);
	map.Accelerate({0, 0, 40}, 50); // z origin on middle of tallest building

	ROS_INFO("%lu obstacles created", map.objects.size());

	Point origin(-5, 20, 15), size({4, 10, 15});
	size = size * inflate;
	Point ptt(origin.x + size.x, origin.y - size.y, origin.z + size.z);
	std::cout << ptt;
	ROS_INFO("Collided? %d", map.PointInMap(ptt) ? 1 : 0);


	// create keypoints
	for(int k = 0; k < map.objects.size(); k++) {
		vec3 size = map.objects[k]->size * inflate;
		Point points[8] = {{size.x, size.y, size.z},
							{-size.x, size.y, size.z},
							{size.x, -size.y, size.z},
							{-size.x, -size.y, size.z},
							{size.x, size.y, -size.z},
							{-size.x, size.y, -size.z},
							{size.x, -size.y, -size.z},
							{-size.x, -size.y, -size.z}};
		const int conn1[] = {0, 0, 0, 1, 1, 2, 2, 3, 4, 4, 5, 6};
		const int conn2[] = {1, 2, 4, 3, 5, 3, 6, 7, 5, 6, 7, 7};

		mat4 transf = FromMat3(map.objects[k]->orientation) *
					  Translation(map.objects[k]->position);

		for(int k2 = 0; k2 < 8; k2++) {
			points[k2] = MultiplyPoint(points[k2], transf);
			if(!map.PointInMap(points[k2]))
			// if(!SimplePointInMap(&map, points[k2]))
				pt.push_back(points[k2]);
		}

		for(int k2 = 0; k2 < 12; k2++) {
			vec3 dir = Normalized(points[conn2[k2]] - points[conn1[k2]]);
			float mlen = Distance(points[conn1[k2]], points[conn2[k2]]);
			float len_inc = mlen / round(mlen / edge_resolution);
			float clen = len_inc;
			while(clen < mlen * 0.99) {  // avoid floating point error
				Point newpt(points[conn1[k2]] + dir * clen);

				if(!map.PointInMap(newpt))
				// if(!SimplePointInMap(&map, newpt))
					pt.push_back(newpt);

				clen += len_inc;
			}
		}
	}
	ROS_INFO("%lu key points created", pt.size());

	// create visibility map
	
}


geometry_msgs::Quaternion quat_from_mat3(mat3 mat) {
	tf2::Quaternion quat;
	mat = Transpose(mat);
	tf2::Matrix3x3 rot(mat._11, mat._12, mat._13,
					  mat._21, mat._22, mat._23,
					  mat._31, mat._32, mat._33);
	rot.getRotation(quat);


	return(tf2::toMsg(quat));
}

void DrawPlanner(ros::Publisher pub) {
	visualization_msgs::Marker marker;

	// plot OBB
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 0.5; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	for(int k = 0; k < planner.map.objects.size(); k++) {
		marker.id = k;
		marker.pose.position.x = planner.map.objects[k]->position.x;
		marker.pose.position.y = planner.map.objects[k]->position.y;
		marker.pose.position.z = planner.map.objects[k]->position.z;
		marker.pose.orientation = quat_from_mat3(planner.map.objects[k]->orientation);
		marker.scale.x = planner.map.objects[k]->size.x * 2.0;
		marker.scale.y = planner.map.objects[k]->size.y * 2.0;
		marker.scale.z = planner.map.objects[k]->size.z * 2.0;

		pub.publish( marker );
	}

	// plot keypoints
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;

	for(int k = 0; k < planner.pt.size(); k++) {
		marker.id++;
		marker.pose.position.x = planner.pt[k].x;
		marker.pose.position.y = planner.pt[k].y;
		marker.pose.position.z = planner.pt[k].z;

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


	ros::Rate loop_rate(10);

	int count = 0;
	while(ros::ok()) {
		DrawPlanner(pub_marker);

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