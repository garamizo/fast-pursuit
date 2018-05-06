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

#define DIST_MAX 1e6


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
	const float inflate = 1.01f;

public:
	std::vector<Point> pt;
	std::vector<std::vector <float>> graph;
	Map map;
	Planner();
	void CreateKeypoints();
	void CreateVisibilityGraph();

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

	CreateKeypoints();

	clock_t begin = clock();
	CreateVisibilityGraph();
	clock_t end = clock();

	ROS_INFO("Elapsed time: %.5f us\n",
		1e6 * (double(end - begin) / CLOCKS_PER_SEC) / 1);

	// ptree = SPT({0, 0, 1});

}


void Planner::CreateKeypoints() {
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
			if(!map.PointInMap(points[k2]) && points[k2].z > 0)
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

				if(!map.PointInMap(newpt) && newpt.z > 0)
				// if(!SimplePointInMap(&map, newpt))
					pt.push_back(newpt);

				clen += len_inc;
			}
		}
	}
	ROS_INFO("%lu key points created", pt.size());
}

void Planner::CreateVisibilityGraph() {
	// create visibility map
	int len = pt.size();
	int count = 0;
	graph.resize(len);
	for(int i = 0; i < len; i++) {
		graph[i].resize(len, -1.0);
		for(int j = i+1; j < len; j++) {

			Line line(pt[i], pt[j]);
			if(!map.Linetest(line)) {
				graph[i][j] = Length(line);
				count++;
			}
		}
	}

	ROS_INFO("%d links created", count);

	// for(int i = 0 ; i < len; i++) {
	// 	for(int j = 0; j < len; j++)
	// 		printf("%3.1f ", graph[i][j]);
	// 	printf("\n");
	// }
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


class SPT {
public:
	Point origin;
	int nnodes;
	std::vector<float> dist;
	std::vector<int> parent;
	std::vector<bool> sptSet;
	const float distmax = DIST_MAX;


	SPT(Point _origin);
	void dijkstra();
	int minDistance();
	void printSolution();
	void printPath(int node);
};

SPT::SPT(Point _origin) {
	origin = _origin;
	nnodes = planner.pt.size();
	dist.resize(nnodes);
	parent.resize(nnodes);
	sptSet.resize(nnodes);

	dijkstra();
}

int SPT::minDistance() {
	// Initialize min value
	int min = INT_MAX, min_index;
	int V = nnodes;

	for (int v = 0; v < V; v++)
		if (sptSet[v] == false && dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}

// A utility function to print 
// the constructed distance
// array
void SPT::printSolution()
{
    int src = nnodes;
    int V = nnodes;
    printf("Vertex\t Distance\tPath");
    for (int i = 0; i < V; i++)
    {
        printf("\n%d -> %d \t\t %f\t\t%d ",
                      V, i, dist[i], src);
        printPath(i);
    }
}

// Function to print shortest
// path from source to j
// using parent array
void SPT::printPath(int node)
{
     
    // Base Case : If j is source
    if (parent[node] == - 1)
        return;
 
    printPath(parent[node]);
 
    printf("%d ", node);
}


// Funtion that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void SPT::dijkstra()
{

	int V = nnodes;
	bool sptSet[nnodes]; // sptSet[i] will true if vertex i is included in shortest
	             // path tree or shortest distance from src to i is finalized

	// Initialize all distances as INFINITE and stpSet[] as false
	for (int i = 0; i < V; i++) {
		dist[i] = distmax;
		sptSet[i] = false;
		parent[i] = -1;
	}

     // Distance of source vertex from itself is always 0
    Line line(origin, {0, 0, 0});
    for(int i = 0; i < V; i++) {
    	line.end = planner.pt[i];
    	if(!planner.map.Linetest(line))
    		dist[i] = Length(line), parent[i] = V;
    }
  
     // Find shortest path for all vertices
     for (int count = 0; count < V-1; count++)
     {
       int u = 	minDistance();
       sptSet[u] = true;
  
       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < V; v++)
         if (!sptSet[v] && planner.graph[u][v] && dist[u] < distmax 
                                       && dist[u]+planner.graph[u][v] < dist[v]) {
         	dist[v] = dist[u] + planner.graph[u][v];
         	parent[v] = u;
         }  
     }
  
     // print the constructed distance array
     printSolution();
}


void DrawGraph(ros::Publisher pub) {
	visualization_msgs::Marker marker;

	marker.id = 1000;
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


	for(int i = 0; i < planner.pt.size(); i++) {
		marker.pose.position.x = planner.pt[i].x;
		marker.pose.position.y = planner.pt[i].y;
		marker.pose.position.z = planner.pt[i].z;
		for(int j = i + 1; j < planner.pt.size(); j++) {
			if(planner.graph[i][j] > 0) {
				marker.id++;
				marker.points[1].x = planner.pt[j].x - planner.pt[i].x;
				marker.points[1].y = planner.pt[j].y - planner.pt[i].y;
				marker.points[1].z = planner.pt[j].z - planner.pt[i].z;
				pub.publish( marker );
			}
		}
	}
	// ROS_INFO("%d links created", marker.id - 1000);
}



void DrawOBB(ros::Publisher pub) {
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
}

void DrawKeypoints(ros::Publisher pub) {
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

		DrawOBB(pub_marker);
		DrawKeypoints(pub_marker);
		DrawGraph(pub_marker);
		
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