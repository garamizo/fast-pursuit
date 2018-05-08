#ifndef _H_PURSUIT_
#define _H_PURSUIT_

#include "Geometry3D.h"
#include <vector>
#include "Map.h"
#include <algorithm>
#include <vector>
#include <iostream>
#include <float.h>


struct PathResult {
	vec3 depart,
		 arrive;
	std::vector<Point> waypts;
	float dist;
};

class SPT {
public:
	float edge_resolution = 2.0f;
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

typedef struct SolverResult {
	std::vector<InterceptionResult> interception;

	int exit_flag;
	int iterations;
} SolverResult;

class Planner {

public:
	Map* map;
	std::vector<SPT> p, e, g;
	std::vector<float> pvel, evel, gvel;
	const float DIST_MIN = 1;
	const int MAX_ITER = 50;
	float edge_resolution = 1.0;
	float cgain = 0.2, ggain = 0.5;

	Planner(Map* _map);
	Planner() {};
	bool SolveInterception(Point point, InterceptionResult& iresult, SolverResult* sresult);
	bool AddPursuer(const Point& point, float vel);
	bool AddEvader(const Point& point, float vel);
	bool AddGoal(const Point& point);
	bool EvaluatePoint(const Point& point, InterceptionResult& intercept);
	std::vector<InterceptionResult> Step();
	void Reconfigure(float _edge_resolution, float _cgain, float _ggain);
};


#endif