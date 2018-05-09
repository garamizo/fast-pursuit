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


void BuildProblem(Map& map, Planner& planner) {
	// build campus map
	float th = -10 * M_PI / 180.0;
	mat3 rot = Transpose(mat3({std::cos(th), -std::sin(th), 0, 
			  std::sin(th), std::cos(th), 0, 
			  0, 0, 1}));
	
	OBB *chem = new OBB({-5, 20, 15}, {4, 10, 15}, rot);
	OBB *mub = new OBB({-30, -15, 2.5}, {10, 10, 2.5}, rot);
	OBB *meem = new OBB({-30, 15, 20}, {5, 5, 20}, rot);

	// map.AddOBB(meem);
	// map.AddOBB(chem);
	// map.AddOBB(mub);
	// map.Accelerate({0, 0, 50}, 50); // z origin on middle of tallest building

	planner.map = &map;
	// planner.AddPursuer({-40, 0, 50}, 1.0f);
	// planner.AddPursuer({15, 40, 50}, 2.0f);
	planner.AddPursuer({20, 25, 30}, 1.0f);
	planner.AddEvader({-50, 30, 30}, 1.6f);
	planner.AddGoal({10, 30, 20});
}


int main(int argc, char **argv) {
	std::srand(std::time(0));
	
	Map map;
	Planner planner;
	BuildProblem(map, planner);

	// Generate points
	vec3 max({70, 70, 50}), min({-70, -30, 1});
	int NDIV = 3;

	std::vector<Point> pts;
	std::vector<float> iterations, costs, constraints;
	vec3 ds({(max.x - min.x) / NDIV, (max.y - min.y) / NDIV, (max.z - min.z) / NDIV});
	
	int trypts = 0;
	for(int i = 0; i < NDIV; i++)
		for(int j = 0; j < NDIV; j++)
			for(int k = 0; k < NDIV; k++) {

				Point point({min.x + i * ds.x,
						     min.y + j * ds.y,
						     min.z + k * ds.z});

				if (!planner.map->PointInMap(point)) {
					trypts++;
					InterceptionResult itcp;
					SolverResult sresult;

					if (planner.SolveInterception(point, itcp, &sresult)) {
						pts.push_back(itcp.point);
						iterations.push_back(sresult.iterations);
						costs.push_back(itcp.cost);
						constraints.push_back(itcp.constraint);
					}
				}
			}

	FILE * pFile;
	char text[200];
	pFile = fopen ("result.csv", "w");

	// write to file
	for (int k = 0; k < costs.size(); k++)
		fprintf(pFile, "%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n", pts[k].x,
			pts[k].y, pts[k].z, iterations[k], costs[k], constraints[k]);

	fclose(pFile);

	return 0;
}