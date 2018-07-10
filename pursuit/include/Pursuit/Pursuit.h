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

class VGraph : public Map {

public:
	std::vector<Point> pt;
	std::vector<std::vector <float>> graph;
	void CreateKeypoints();
	void CreateVisibilityGraph();

	void Rewire();
	float edge_resolution = 1.0f;
	float inflate = 1.01f;
	const float DIST_MAX = 1e6;
	int link_count;
};

class SPT {  // TODO convert to struct
public:

	Point root;
	std::vector<float> dist;
	std::vector<int> parent;
	std::vector<bool> sptSet;
	
	VGraph* map;

	SPT(VGraph* _map);
	void Rewire(const Point& _root);
	int minDistance();
	void printSolution();
	void printPath(int node);
	bool findPath(const Point& dest, PathResult& result);
	void printPathResult(const PathResult& result);
};


typedef struct InterceptionResult {
	Point point;
	SPT *p, *e, *g;
	float cost, constraint;
	vec3 costd, constraintd;
	PathResult ppath, epath, gpath;
	int exit_flag;
	int iterations;
} InterceptionResult;

typedef struct SolverResult {
	std::vector<InterceptionResult> interception;

	int exit_flag;
	int iterations;
} SolverResult;

class Planner {

private:


public:
	Planner& operator=(const Planner&);

	std::vector<SPT> p, e, g;
	std::vector<float> pvel, evel, gvel;
	VGraph* map;
	const float TOL_CONSTRAINT = 0.1,
				TOL_UPDATE = 1.0;
	const int MAX_ITER = 30;
	float cgain = 0.15, ggain = 17.5/2, momentum = 0.3;

	Planner(VGraph* _map);
	Planner(int nobs, int np, float edge_resolution);
	Planner(int nobs, int np) { Planner(nobs, np, 1.0); }
	bool SolveInterception(Point point, InterceptionResult& outResult);
	bool SolveInterceptionAnalysis(Point point, InterceptionResult& outResult, SolverResult* sresult);
	bool AddPursuer(const Point& point, float vel);
	bool AddEvader(const Point& point, float vel);
	bool AddGoal(const Point& point);
	bool EvaluatePoint(const Point& point, InterceptionResult& intercept);
	std::vector<InterceptionResult> Step();
};

std::ostream& operator<<(std::ostream& os, Planner planner);
std::ostream& operator<<(std::ostream& os, SPT spt);


#endif