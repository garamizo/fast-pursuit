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
	float edge_resolution = 3.0f;
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


typedef struct InterceptionResult {
	Point point;
	SPT *p, *e, *g;
	float cost, constraint;
	vec3 costd, constraintd;
	PathResult ppath, epath, gpath;
} InterceptionResult;

class Planner {

public:
	Map* map;
	std::vector<SPT> p, e, g;
	std::vector<float> pvel, evel, gvel;
	const float DIST_MIN = 1e-1;

	Planner(Map* _map);
	bool SolveInterception(Point point, InterceptionResult& result, std::vector<Point>* sol);
	bool AddPursuer(const Point& point, float vel);
	bool AddEvader(const Point& point, float vel);
	bool AddGoal(const Point& point);
	bool EvaluatePoint(const Point& point, InterceptionResult& intercept);
	std::vector<InterceptionResult> Step();
};

#endif