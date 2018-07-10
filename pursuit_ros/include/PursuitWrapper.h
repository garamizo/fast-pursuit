#ifndef _H_PURSUITWRAPPER_
#define _H_PURSUITWRAPPER_

#include "ros/ros.h"
#include "pursuit_msgs/Interception.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "Pursuit.h"
#include "vectors.h"

#include "sensor_msgs/PointCloud2.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/algorithm/clamp.hpp>



class PursuitWrapper {

	ros::NodeHandle nh;
	ros::Publisher pub_marker,
				   pub_cloud,
				   pub_cloudsol,
				   pub_ipt;

    ros::Subscriber sub1,
    				sub2;

    pursuit_msgs::Interception msg;
    visualization_msgs::Marker ipt;
    int ipt_count;

public:
	Planner planner;

    PursuitWrapper(Planner _planner);
    PursuitWrapper();
	void setup();
	void LoadROSParam();
    void callback1(const nav_msgs::Odometry::ConstPtr& msg);
    void callback2(const nav_msgs::Odometry::ConstPtr& msg);
	void DrawAgents();
	void DrawObstacles();
	void DrawKeypoints();
	void DrawPath(const PathResult& result, int id, vec3 color);
	void BuildPointCloud();
	void BuildInterceptCloud();
	void DrawConvergence();
	void DrawTree(SPT spt);
	void step();
};

bool getHeatMapColor(float value, float *red, float *green, float *blue);
geometry_msgs::Quaternion quat_from_mat3(mat3 mat);

#endif