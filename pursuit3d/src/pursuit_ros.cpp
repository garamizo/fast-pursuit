#include "ros/ros.h"
#include "pursuit3d/Interception.h"
#include "nav_msgs/Odometry.h"

#include "Map.h"
#include "Pursuit.h"
#include "vectors.h"


class Listener 
{

  public:
  	Map map;
    Planner planner;

    Listener();
    void callback1(const nav_msgs::Odometry::ConstPtr& msg);
    void callback2(const nav_msgs::Odometry::ConstPtr& msg);
};

void Listener::callback1(const nav_msgs::Odometry::ConstPtr& msg)
{
    planner.e[0].root.x = msg->pose.pose.position.x;
    planner.e[0].root.y = msg->pose.pose.position.y;
    planner.e[0].root.z = msg->pose.pose.position.z;

    ROS_INFO_THROTTLE(1, "(%.2f, %.2f, %.2f)\n", planner.e[0].root.x, planner.e[0].root.y, planner.e[0].root.z);
}

void Listener::callback2(const nav_msgs::Odometry::ConstPtr& msg)
{
    planner.p[0].root.x = msg->pose.pose.position.x;
    planner.p[0].root.y = msg->pose.pose.position.y;
    planner.p[0].root.z = msg->pose.pose.position.z;
}

Listener::Listener() {
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
	// map.AddOBB(mub);
	// map.Accelerate({0, 0, 50}, 50); // z origin on middle of tallest building

	planner.map = &map;
	planner.AddPursuer({20, 0, 50}, 1.0f);
	// planner.AddPursuer({15, 40, 50}, 2.0f);
	// planner.AddPursuer({50, 50, 10}, 1.0f);
	planner.AddEvader({-50, 30, 30}, 1.4f);
	planner.AddGoal({0, 0, 0});
}


int main(int argc, char **argv) {
	std::srand(std::time(0));

	ros::init(argc, argv, "pursuit");
	ros::NodeHandle nh;

	Listener listener;

	ros::Subscriber sub1 = nh.subscribe<nav_msgs::Odometry>("/ardrone_1/ground_truth/state", 1, &Listener::callback1, &listener);
	ros::Subscriber sub2 = nh.subscribe<nav_msgs::Odometry>("/ardrone_2/ground_truth/state", 1, &Listener::callback2, &listener);

	ros::Publisher pub = nh.advertise<pursuit3d::Interception>("interception", 10000);
	
	// Generate points
	vec3 max({70, 70, 50}), min({-70, -30, 1});
	vec3 ds = max - min;

	pursuit3d::Interception msg;
	
	while (ros::ok()) {

		Point point({min.x + rand() * ds.x / RAND_MAX,
				     min.y + rand() * ds.y / RAND_MAX,
				     min.z + rand() * ds.z / RAND_MAX});

		if (!listener.planner.map->PointInMap(point)) {
			InterceptionResult itcp;

			if (listener.planner.SolveInterception(point, itcp, NULL)) {
				msg.point.x = itcp.point.x;
				msg.point.y = itcp.point.y;
				msg.point.z = itcp.point.z;
				msg.cost = itcp.cost;
				msg.constraint = itcp.constraint;
				msg.pidx = int(itcp.p - &listener.planner.p[0]);

				pub.publish(msg);
			}
		}

		ros::spinOnce();
	}

	return 0;
	
}