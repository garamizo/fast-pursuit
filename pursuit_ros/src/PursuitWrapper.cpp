#include "PursuitWrapper.h"


PursuitWrapper::PursuitWrapper(Planner _planner) : planner(_planner), nh("~") {
	setup();
}

PursuitWrapper::PursuitWrapper() : planner(new VGraph()), nh("~") {
	LoadROSParam();
	setup();
}

void PursuitWrapper::setup() {

	pub_marker = nh.advertise<visualization_msgs::Marker>("/marker_map", 10000);
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 10000);
	pub_cloudsol = nh.advertise<sensor_msgs::PointCloud2>("/cloud_intercept", 1000);
	pub_ipt = nh.advertise<pursuit_msgs::Interception>("/interception", 10000);

	sub1 = nh.subscribe<nav_msgs::Odometry>("/pursuer1", 1, &PursuitWrapper::callback1, this);
	sub2 = nh.subscribe<nav_msgs::Odometry>("/evader1", 1, &PursuitWrapper::callback2, this);
	
	DrawObstacles();
	DrawKeypoints();
	DrawAgents();

	ipt.header.frame_id = "map";
	ipt.header.stamp = ros::Time();
	ipt.ns = "solution";
	ipt.type = visualization_msgs::Marker::SPHERE_LIST;
	ipt.action = visualization_msgs::Marker::ADD;
	ipt.color.a = 1.0; // Don't forget to set the alpha!
	ipt.color.r = 1.0;
	ipt.color.g = 0.4;
	ipt.color.b = 0.0;
	ipt.scale.x = ipt.scale.y = ipt.scale.z = 1.0;
	ipt.pose.orientation.w = 1;
	ipt.points.resize(100);
	ipt_count = 0; 

	// planner.map->Accelerate(); // z origin on middle of tallest building
}

void PursuitWrapper::LoadROSParam() {

	std::map<std::string, float> map_params;
	ROS_FATAL_COND(!nh.getParam("map", map_params), "No map param found");
	planner.map->edge_resolution = map_params["edge_resolution"];
	planner.map->bounds = FromMinMax({map_params["xmin"], map_params["ymin"], map_params["zmin"]},
							{map_params["xmax"], map_params["ymax"], map_params["zmax"]});

	std::map<std::string, float> o;
	for (int i = 1; nh.hasParam(std::string("obstacle") + std::to_string(i)); i++) {
		nh.getParam(std::string("obstacle") + std::to_string(i), o);

		planner.map->AddOBB(OBB({o["x"], o["y"], o["z"]},
					   {o["lx"], o["ly"], o["lz"]},
					   Rotation3x3(o["rx"], o["ry"], o["rz"])));
	}
	planner.map->Rewire();

	std::map<std::string, float> g;
	ROS_FATAL_COND(!nh.getParam("goal1", g), "goal param not set");
	planner.AddGoal({g["x"], g["y"], g["z"]});

	std::map<std::string, float> e;
	ROS_FATAL_COND(!nh.getParam("evader1", e), "evader param not set");
	planner.AddEvader({e["x"], e["y"], e["z"]}, e["speed"]);

	std::map<std::string, float> p;
	for (int i = 1; nh.hasParam(std::string("pursuer") + std::to_string(i)); i++) {
		nh.getParam(std::string("pursuer") + std::to_string(i), p);
		planner.AddPursuer({p["x"], p["y"], p["z"]}, p["speed"]);
	}

	std::map<std::string, float> solver;
	ROS_FATAL_COND(!nh.getParam("solver", solver), "No solver param found");
	planner.cgain = solver["cgain"];
	planner.ggain = solver["ggain"];
	planner.momentum = solver["momentum"];
}

void PursuitWrapper::callback1(const nav_msgs::Odometry::ConstPtr& msg) {
	planner.e[0].Rewire({float(msg->pose.pose.position.x),
						 float(msg->pose.pose.position.y),
						 float(msg->pose.pose.position.z)});
}

void PursuitWrapper::callback2(const nav_msgs::Odometry::ConstPtr& msg) {
	planner.p[0].Rewire({float(msg->pose.pose.position.x),
						 float(msg->pose.pose.position.y),
						 float(msg->pose.pose.position.z)});
}

void PursuitWrapper::DrawAgents() {
	ros::Duration(1).sleep();

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
		text.text = std::string("P") + std::to_string(k+1);
	
		pub_marker.publish( marker );
		pub_marker.publish( text );
	}

	// EVADERS
	marker.color.r = 1;
	marker.color.g = 0;
	marker.color.b = 0;
	// marker.color.r = 0.55;
	// marker.color.g = 0.75;
	// marker.color.b = 0.25;

	for(int k = 0; k < planner.e.size(); k++) {
		marker.id++;
		marker.pose.position.x = planner.e[k].root.x;
		marker.pose.position.y = planner.e[k].root.y;
		marker.pose.position.z = planner.e[k].root.z;

		text.id++;
		text.pose.position.x = planner.e[k].root.x;
		text.pose.position.y = planner.e[k].root.y + 5;
		text.pose.position.z = planner.e[k].root.z + 2.5;
		// text.text = std::string("E") + std::to_string(k+1);
		text.text = std::string("E");
	
		pub_marker.publish( marker );
		pub_marker.publish( text );
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
		text.text = std::string("T");
		// text.text = std::string("G") + std::to_string(k+1);
	
		pub_marker.publish( marker );
		pub_marker.publish( text );
	}
}

void PursuitWrapper::DrawObstacles() {
	ros::Duration(1).sleep();
	visualization_msgs::Marker marker;

	// plot OBB
	marker.id = 0;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "obstacles_opa";
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 0.9; // Don't forget to set the alpha!
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;

	for(int k = 0; k < planner.map->objects.size(); k++) {
		marker.id = k;
		marker.pose.position.x = planner.map->objects[k].position.x;
		marker.pose.position.y = planner.map->objects[k].position.y;
		marker.pose.position.z = planner.map->objects[k].position.z;
		marker.pose.orientation = quat_from_mat3(planner.map->objects[k].orientation);
		marker.scale.x = planner.map->objects[k].size.x * 2.0;
		marker.scale.y = planner.map->objects[k].size.y * 2.0;
		marker.scale.z = planner.map->objects[k].size.z * 2.0;

		pub_marker.publish( marker );
	}
}

void PursuitWrapper::DrawKeypoints() {
	ros::Duration(1).sleep();
	visualization_msgs::Marker marker;

	// plot keypoints
	marker.id = 0;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "keypoints";
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	marker.points.resize(planner.map->pt.size());

	for(int k = 0; k < planner.map->pt.size(); k++) {
		marker.points[k].x = planner.map->pt[k].x;
		marker.points[k].y = planner.map->pt[k].y;
		marker.points[k].z = planner.map->pt[k].z;
	}
	pub_marker.publish( marker );
}

void PursuitWrapper::DrawPath(const PathResult& result, int id, vec3 color) {
	ros::Duration(1).sleep();
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
		pub_marker.publish( marker );
	}
	// ROS_INFO("%d links created", marker.id - 1000);
}

void PursuitWrapper::BuildPointCloud() {
	ros::Duration(1).sleep();
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	
	vec3 max = GetMax(planner.map->bounds), min = GetMin(planner.map->bounds);
	int NDIV = 70;
	vec3 ds = (max - min) / NDIV;
	std::vector<float> cost;
	float mincost = 1e10, maxcost = -1;

	for(int i = 0; i < NDIV; i++) {
		for(int j = 0; j < NDIV; j++)
			for(int k = 0; k < NDIV; k++) {
				pcl::PointXYZRGBA point;
		
				Point eval_point({min.x + i * ds.x,
						     min.y + j * ds.y,
						     min.z + k * ds.z});

				InterceptionResult itcp;
				if (!planner.EvaluatePoint(eval_point, itcp))
					continue;


				if (fabs(itcp.constraint) < 0.05/*&& itcp.cost < 70.0*/) {

					point.x = eval_point.x;
					point.y = eval_point.y;
					point.z = eval_point.z;

					mincost = fminf(mincost, itcp.cost);
					maxcost = fmaxf(maxcost, itcp.cost);

					cost.push_back(itcp.cost);
					cloud.push_back(point);
				}

			}
		ROS_INFO_THROTTLE(3, "Progress: %.1f %%", 100*(i+1.0)/NDIV);
	}

	for (int i = 0; i < cost.size(); i++) {
		float cmap = 1-(cost[i] - mincost)/(maxcost - mincost);
		vec3 color;
		getHeatMapColor(cmap, &color.x, &color.y, &color.z);

		cloud[i].r = 255 * color.x;
		cloud[i].g = 255 * color.y;
		cloud[i].b = 255 * color.z;
		cloud[i].a = 255;
	}
	printf("max: %f\nmin: %f", maxcost, mincost);

	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg<pcl::PointXYZRGBA>(cloud, cloud_msg);
	cloud_msg.header.frame_id = "map";
	cloud_msg.header.stamp = ros::Time();

	pub_cloud.publish( cloud_msg );
	ROS_INFO("Done with cloud!");
}

void PursuitWrapper::BuildInterceptCloud() {
	ros::Duration(1).sleep();

	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	pcl::PointXYZRGBA point;

	// plan an interception
	std::vector<InterceptionResult> sols = planner.Step();

	for(int i = 0; i < sols.size(); i++) {

		point.x = sols[i].point.x;
		point.y = sols[i].point.y;
		point.z = sols[i].point.z;

		point.r = 128;
		point.g = 0;
		point.b = 128;
		point.a = 255;

		cloud.push_back(point);
	}

	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg<pcl::PointXYZRGBA>(cloud, cloud_msg);
	cloud_msg.header.frame_id = "map";
	cloud_msg.header.stamp = ros::Time();

	pub_cloudsol.publish( cloud_msg );
}

void PursuitWrapper::DrawConvergence() {
	ros::Duration(1).sleep();

	visualization_msgs::Marker marker, start_point, marker_square;

	start_point.header.frame_id = "map";
	start_point.header.stamp = ros::Time();
	start_point.ns = "start_pt";
	start_point.type = visualization_msgs::Marker::SPHERE_LIST;
	start_point.action = visualization_msgs::Marker::ADD;
	start_point.color.a = 1.0; // Don't forget to set the alpha!
	start_point.color.r = 1.0;
	start_point.color.g = 0.5;
	start_point.color.b = 0.0;
	start_point.scale.x = start_point.scale.y = start_point.scale.z = 1.0;
	start_point.pose.orientation.w = 1;

	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "convergence";
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.5;
	marker.color.g = 0.4;
	marker.color.b = 0.2;
	marker.scale.x = 0.2;
	marker.pose.orientation.w = 1;

	marker_square = marker;
	marker_square.type = visualization_msgs::Marker::CUBE_LIST;
	marker_square.ns = "squares";
	marker_square.scale.x = marker_square.scale.y = marker_square.scale.z = 0.4;

	// Test multiple points
	vec3 max = GetMax(planner.map->bounds), min = GetMin(planner.map->bounds);
	int NDIV = 3;

	start_point.points.resize(NDIV*NDIV*NDIV);
	vec3 ds = (max - min) / NDIV;
	
	for(int i = 0; i < NDIV; i++)
		for(int j = 0; j < NDIV; j++)
			for(int k = 0; k < NDIV; k++) {

				Point point({min.x + (i + 0.5f) * ds.x,
						     min.y + (j + 0.5f) * ds.y,
						     min.z + (k + 0.5f) * ds.z});

				start_point.points[k+NDIV*j+NDIV*NDIV*i].x = point.x;
				start_point.points[k+NDIV*j+NDIV*NDIV*i].y = point.y;
				start_point.points[k+NDIV*j+NDIV*NDIV*i].z = point.z;

				if (!planner.map->PointInMap(point)) {

					InterceptionResult itcp;
					SolverResult sresult;
					if (planner.SolveInterceptionAnalysis(point, itcp, &sresult)) {
						marker.points.resize(sresult.interception.size());
						for (int p = 0; p < sresult.interception.size(); p++) {

							marker.points[p].x = sresult.interception[p].point.x;
							marker.points[p].y = sresult.interception[p].point.y;
							marker.points[p].z = sresult.interception[p].point.z;
						}
						marker.id++;
						pub_marker.publish( marker );

						ros::Duration(0.05).sleep();
						marker_square.points = marker.points;
						marker_square.id++;
						pub_marker.publish( marker_square );
					}
				}
				ros::spinOnce();
			}

	pub_marker.publish(start_point);
	ROS_INFO("Done with convergence");
}

void PursuitWrapper::DrawTree(SPT spt) {
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
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.pose.orientation.w = 1;
	marker.points.resize(2);

	for(int i = 0; i < planner.map->pt.size(); i++) {
		if(spt.parent[i] < 0)
			continue;

		marker.pose.position.x = planner.map->pt[i].x;
		marker.pose.position.y = planner.map->pt[i].y;
		marker.pose.position.z = planner.map->pt[i].z;

		if(spt.parent[i] < planner.map->pt.size()) {
			marker.points[1].x = planner.map->pt[spt.parent[i]].x - planner.map->pt[i].x;
			marker.points[1].y = planner.map->pt[spt.parent[i]].y - planner.map->pt[i].y;
			marker.points[1].z = planner.map->pt[spt.parent[i]].z - planner.map->pt[i].z;
		}
		else {
			marker.points[1].x = spt.root.x - planner.map->pt[i].x;
			marker.points[1].y = spt.root.y - planner.map->pt[i].y;
			marker.points[1].z = spt.root.z - planner.map->pt[i].z;
		}

		marker.id++;
		pub_marker.publish( marker );
		ros::Duration(0.01).sleep();
	}
}

void PursuitWrapper::step() {

	vec3 min = GetMin(planner.map->bounds);
	vec3 ds = planner.map->bounds.size * 2.0f;

	Point point({min.x + rand() * ds.x / RAND_MAX,
			     min.y + rand() * ds.y / RAND_MAX,
			     min.z + rand() * ds.z / RAND_MAX});

	if (!planner.map->PointInMap(point)) {
		InterceptionResult itcp;

		if (planner.SolveInterception(point, itcp)) {
			msg.point.x = itcp.point.x;
			msg.point.y = itcp.point.y;
			msg.point.z = itcp.point.z;
			msg.cost = itcp.cost;
			msg.constraint = itcp.constraint;
			msg.pidx = int(itcp.p - &planner.p[0]);

			pub_ipt.publish(msg);

			// plot results and agents
			ipt.points[ipt_count].x = itcp.point.x;
			ipt.points[ipt_count].y = itcp.point.y;
			ipt.points[ipt_count].z = itcp.point.z;
			ipt_count++;

			if (ipt_count == 100) {  // throttle for performance
				ipt_count = 0;
				pub_marker.publish(ipt);
			}
		}
	}
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


geometry_msgs::Quaternion quat_from_mat3(mat3 mat) {
	tf2::Quaternion quat;
	mat = Transpose(mat);
	tf2::Matrix3x3 rot(mat._11, mat._12, mat._13,
					  mat._21, mat._22, mat._23,
					  mat._31, mat._32, mat._33);
	rot.getRotation(quat);


	return(tf2::toMsg(quat));
}
