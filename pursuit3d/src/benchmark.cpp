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
	
	Map map;
	Planner planner;
	BuildProblem(map, planner);

	std::vector<InterceptionResult> sols;
	FILE * pFile;
	char text[200];
	
	float cgain[] = {0.05, 0.1, 0.2, 0.3, 0.4};
	float ggain[] = {0.1, 0.3, 0.5, 1.0, 1.5};
	float edge_res[] = {3.0, 2.5, 2.0, 1.5, 1.0, 0.5, 0.3};
	for(int i = 0; i < 7; i++) {
		std::cout << "\nds\tcgain\tggain\t# sols\tRatio\tTime\t\tMin\tMax\tMean\tOpt <\t# Links\n";
		sprintf(text, "results%d.csv", i);
		pFile = fopen (text, "w");

		// planner.Reconfigure(2.0, 0.2, ggain[i]);
		// planner.Reconfigure(2.0, cgain[i], 0.5);
		planner.Reconfigure(edge_res[i], 0.2, 0.5);

		for(int j = 0; j < 10; j++) {
			sols = planner.Step();

			for (int k = 0, size = sols.size(); k < size; k++)
				fprintf(pFile, "%.10f\t%.10f\t%.10f\t%.10f\t%.10f\n", sols[k].point.x,
					sols[k].point.y, sols[k].point.z, sols[k].cost, sols[k].constraint);
		}
		fclose(pFile);
	}

	return 0;
}