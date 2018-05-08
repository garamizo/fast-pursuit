#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "visualization_msgs/Marker.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sstream>
#include <numeric>

#include "matrices.h"
#include "vectors.h"
#include "Geometry3D.h"
#include "Map.h"
#include "Pursuit.h"
#include <iostream>
#include <cmath>
#include <cfloat>
#include <ctime>
#include <boost/algorithm/clamp.hpp>

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

	marker.id = 0;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "graph";
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
	marker.ns = "obstacles";
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.3;
	marker.color.g = 0.3;
	marker.color.b = 0.3;

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
	marker.id = 0;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "keypoints";
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.3;

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
	marker.id = 0;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "keypoints_labels";
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

	marker.id = 0;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "tree";
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
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

void DrawPath(const PathResult& result, ros::Publisher pub, int id, vec3 color) {
	visualization_msgs::Marker marker;

	marker.id = id;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "path";
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = color.x;
	marker.color.g = color.y;
	marker.color.b = color.z;
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

void DrawRobots(const Planner& planner, ros::Publisher pub) {
	visualization_msgs::Marker marker;
	visualization_msgs::Marker text;

	// plot OBB
	marker.id = 0;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "robots";
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;

	marker.scale.x = 3.0;
	marker.scale.y = 3.0;
	marker.scale.z = 1.0;
	marker.pose.orientation.w = 1.0;

	// plot text
	text.id = 0;
	text.header.frame_id = "map";
	text.header.stamp = ros::Time();
	text.ns = "robots_labels";
	text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text.action = visualization_msgs::Marker::ADD;
	text.color.a = 1.0; // Don't forget to set the alpha!
	text.color.r = 0.0;
	text.color.g = 0.0;
	text.color.b = 0.0;
	text.scale.z = 5.0;
	text.pose.orientation.w = 1.0;

	for(int k = 0; k < planner.p.size(); k++) {
		marker.id++;
		marker.pose.position.x = planner.p[k].root.x;
		marker.pose.position.y = planner.p[k].root.y;
		marker.pose.position.z = planner.p[k].root.z;

		text.id++;
		text.pose.position.x = planner.p[k].root.x;
		text.pose.position.y = planner.p[k].root.y - 5;
		text.pose.position.z = planner.p[k].root.z + 2.5;
		text.text = std::string("P") + std::to_string(k);
	
		pub.publish( marker );
		pub.publish( text );
	}

	// EVADERS
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	for(int k = 0; k < planner.e.size(); k++) {
		marker.id++;
		marker.pose.position.x = planner.e[k].root.x;
		marker.pose.position.y = planner.e[k].root.y;
		marker.pose.position.z = planner.e[k].root.z;

		text.id++;
		text.pose.position.x = planner.e[k].root.x;
		text.pose.position.y = planner.e[k].root.y - 5;
		text.pose.position.z = planner.e[k].root.z + 2.5;
		text.text = std::string("E") + std::to_string(k);
	
		pub.publish( marker );
		pub.publish( text );
	}

	// GOALS
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.scale.x = 5.0;
	marker.scale.y = 5.0;
	marker.scale.z = 5.0;

	for(int k = 0; k < planner.g.size(); k++) {
		marker.id++;
		marker.pose.position.x = planner.g[k].root.x;
		marker.pose.position.y = planner.g[k].root.y;
		marker.pose.position.z = planner.g[k].root.z;

		text.id++;
		text.pose.position.x = planner.g[k].root.x;
		text.pose.position.y = planner.g[k].root.y - 5;
		text.pose.position.z = planner.g[k].root.z + 2.5;
		text.text = std::string("G") + std::to_string(k);
	
		pub.publish( marker );
		pub.publish( text );
	}
}

void DrawConvergence(const std::vector<Point>& pts, ros::Publisher pub, int id) {
	visualization_msgs::Marker marker;

	marker.id = id;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "convergence";
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.5;
	marker.color.g = 0.4;
	marker.color.b = 0.2;
	marker.scale.x = 0.2;
	marker.scale.y = 0.7;
	marker.scale.z = 1.0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;
	marker.points.resize(2);


	for(int i = 0; i < pts.size() - 1; i++) {
		marker.pose.position.x = pts[i].x;
		marker.pose.position.y = pts[i].y;
		marker.pose.position.z = pts[i].z;

		marker.id++;
		marker.points[1].x = pts[i+1].x - pts[i].x;
		marker.points[1].y = pts[i+1].y - pts[i].y;
		marker.points[1].z = pts[i+1].z - pts[i].z;
		pub.publish( marker );
	}
	// ROS_INFO("Done with convergence");
}

bool getHeatMapColor(float value, float *red, float *green, float *blue)
{
  const int NUM_COLORS = 4;
  static float color[NUM_COLORS][3] = { {0,0,1}, {0,1,0}, {1,1,0}, {1,0,0} };
    // A static array of 4 colors:  (blue,   green,  yellow,  red) using {r,g,b} for each.
 
  int idx1;        // |-- Our desired color will be between these two indexes in "color".
  int idx2;        // |
  float fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.
 
  if(value <= 0)      {  idx1 = idx2 = 0;            }    // accounts for an input <=0
  else if(value >= 1)  {  idx1 = idx2 = NUM_COLORS-1; }    // accounts for an input >=0
  else
  {
    value = value * (NUM_COLORS-1);        // Will multiply value by 3.
    idx1  = floor(value);                  // Our desired color will be after this index.
    idx2  = idx1+1;                        // ... and before this index (inclusive).
    fractBetween = value - float(idx1);    // Distance between the two indexes (0-1).
  }
 
  *red   = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0];
  *green = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1];
  *blue  = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2];
}


void BuildPointCloud(Planner& planner, ros::Publisher pub) {
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	
	Point min(-50.0, -10.0, 0.0), max(20.0, 50.0, 60.0);
	int NDIV = 70;
	vec3 ds({(max.x - min.x) / NDIV, (max.y - min.y) / NDIV, (max.z - min.z) / NDIV});

	for(int i = 0; i < NDIV; i++)
		for(int j = 0; j < NDIV; j++)
			for(int k = 0; k < NDIV; k++) {
				pcl::PointXYZRGBA point;
		
				Point eval_point({min.x + i * ds.x,
						     min.y + j * ds.y,
						     min.z + k * ds.z});

				InterceptionResult itcp;
				bool valid = planner.EvaluatePoint(eval_point, itcp);

				if (valid && fabs(itcp.constraint) < 1.0 && itcp.cost < 70.0) {
					point.x = eval_point.x;
					point.y = eval_point.y;
					point.z = eval_point.z;

					float cmin = 24.54, cmax = 50;
					float cmap = boost::algorithm::clamp(1-(itcp.cost - cmin)/cmax, 0.0, 1.0);
					vec3 color;
					getHeatMapColor(cmap, &color.x, &color.y, &color.z);

					point.r = 255 * color.x;
					point.g = 255 * color.y;
					point.b = 255 * color.z;
					point.a = 255;

					cloud.push_back(point);
				}

			}

	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg<pcl::PointXYZRGBA>(cloud, cloud_msg);
	cloud_msg.header.frame_id = "map";
	cloud_msg.header.stamp = ros::Time();

	pub.publish( cloud_msg );
	ROS_INFO("Done with cloud!");
}

void DrawPoints(std::vector<Point> pts, std::string ns, vec3 color, ros::Publisher pub) {
	visualization_msgs::Marker marker;

	// plot keypoints
	marker.id = 0;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = ns;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = color.x;
	marker.color.g = color.y;
	marker.color.b = color.z;
	marker.scale.x = 2;
	marker.scale.y = 2;
	marker.scale.z = 2;

	for(int k = 0; k < pts.size(); k++) {
		marker.id++;
		marker.pose.position.x = pts[k].x;
		marker.pose.position.y = pts[k].y;
		marker.pose.position.z = pts[k].z;

		pub.publish( marker );
	}
}


void BuildInterceptCloud(std::vector<InterceptionResult> sols, ros::Publisher pub) {

	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	pcl::PointXYZRGBA point;

	for(int i = 0; i < sols.size(); i++) {

		point.x = sols[i].point.x;
		point.y = sols[i].point.y;
		point.z = sols[i].point.z;

		point.r = 255;
		point.g = 100;
		point.b = 30;
		point.a = 255;

		cloud.push_back(point);
	}

	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg<pcl::PointXYZRGBA>(cloud, cloud_msg);
	cloud_msg.header.frame_id = "map";
	cloud_msg.header.stamp = ros::Time();

	pub.publish( cloud_msg );
}


void BuildProblem(Map& map, Planner& planner) {
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

	planner.map = &map;
	planner.AddPursuer({-40, 0, 10}, 1.0f);
	planner.AddPursuer({15, 40, 2}, 1.0f);
	planner.AddPursuer({20, 25, 30}, 1.5f);
	planner.AddEvader({-30, 30, 30}, 1.2f);
	planner.AddGoal({10, 20, 1});
}


int main(int argc, char **argv) {
	std::srand(std::time(0));

	Map map;
	Planner planner;
	BuildProblem(map, planner);


	ros::init(argc, argv, "pursuit");
	ros::NodeHandle n;

	ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>("marker_map", 10000);
	ros::Publisher pub_cloud = n.advertise<sensor_msgs::PointCloud2>("cloud", 10000);
	// ros::Publisher pub_cloud2 = n.advertise<sensor_msgs::PointCloud2>("cloud_intercept", 1000);

	// Test multiple points
	std::vector<InterceptionResult> sols = planner.Step();
	std::vector<Point> goodpts, badpts;
	for (int i = 0; i < sols.size(); i++)
	{
		if (fabs(sols[i].constraint) < 0.5) {
			goodpts.push_back(sols[i].point);
		}
		else {
			badpts.push_back(sols[i].point);
		}
	}
	ROS_INFO("%lu good and %lu bad solutions found", goodpts.size(), badpts.size());

	DrawObstacles(map, pub_marker);
	ros::Duration(0.5).sleep();

	DrawKeypoints(planner.p[0], pub_marker);
	ros::Duration(0.5).sleep();

	DrawRobots(planner, pub_marker);
	ros::Duration(0.5).sleep();

	DrawPoints(goodpts, "good", {1, 0.7, 0.1}, pub_marker);
	ros::Duration(0.5).sleep();

	DrawPoints(badpts, "bad", {0, 0, 0}, pub_marker);
	ros::Duration(0.5).sleep();

	BuildPointCloud(planner, pub_cloud);

	ros::spin();
	return 0;
}