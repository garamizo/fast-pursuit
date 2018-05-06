#ifndef _H_PURSUIT_
#define _H_PURSUIT_

#include "Geometry3D.h"
#include <vector>
#include "Map.h"
#include <algorithm>
#include <vector>
#include <iostream>


struct PathResult {
	vec3 depart,
		 arrive;
	std::vector<Point> waypts;
	float dist;
};

class SPT {
public:
	float edge_resolution = 5.0f;
	float inflate = 1.01f;
	float DIST_MAX = 1e6;

	Point root;  // change to root
	std::vector<Point> pt;
	std::vector<std::vector <float>> graph;
	std::vector<float> dist;
	std::vector<int> parent;
	std::vector<bool> sptSet;
	
	Map* map;
	int link_count;

	SPT(Map* _map, const Point& _root);
	void Rewire(const Point& _root);
	int minDistance();
	void printSolution();
	void printPath(int node);
	bool findPath(const Point& dest, PathResult& result);
	void printPathResult(const PathResult& result);
	void CreateKeypoints();
	void CreateVisibilityGraph();
};


struct InterceptionResult {
	Point position;
	SPT* p, e;
	float atdist;
};

class Planner {

	const float edge_resolution = 5.0f;
	const float inflate = 1.01f;

public:
	Map* map;
	std::vector<SPT> p, e;

	Planner(Map* _map);
	bool CalculateInterception(InterceptionResult& result);
	bool AddPursuer(const Point& point);
	bool AddEvader(const Point& point);
};

#endif