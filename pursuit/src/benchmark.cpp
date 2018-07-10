#include "vectors.h"
#include "Pursuit.h"
#include <iostream>
#include <chrono>


using namespace std::chrono;


void usage() {
	std::cout << "Print benchmark results to screen\n\nUsage:\n\n"
			  << "\tbenchmark NODE_DENSITY SEED\n"
			  << "\tbenchmark 1.0 50\n";
}


int main(int argc, char **argv) {

	if (argc != 3) {
		usage();
		return -1;
	}
	float edge_resolution = 1.0 / atof(argv[1]);
	int seed = atoi(argv[2]);
	int nobs = 20;
	int np = 3;

	std::srand(seed);
	Planner planner(nobs, np, edge_resolution);

	// Generate points
	vec3 max = GetMax(planner.map->bounds), min = GetMin(planner.map->bounds);
	int NDIV = 5;
	vec3 ds = (max - min) / NDIV;

	std::vector<Point> pts;
	std::vector<int> iterations;
	std::vector<float> costs, constraints;
	std::vector<long int> sexecution;

	for(int i = 0; i < NDIV; i++)
		for(int j = 0; j < NDIV; j++)
			for(int k = 0; k < NDIV; k++) {

				Point point({min.x + i * ds.x,
						     min.y + j * ds.y,
						     min.z + k * ds.z});

				if (!planner.map->PointInMap(point)) {
					InterceptionResult itcp;
					high_resolution_clock::time_point t1 = high_resolution_clock::now();

					if (planner.SolveInterception(point, itcp)) {
						high_resolution_clock::time_point t2 = high_resolution_clock::now();
						auto duration = duration_cast<microseconds>( t2 - t1 ).count();

						pts.push_back(itcp.point);
						iterations.push_back(itcp.iterations);
						costs.push_back(itcp.cost);
						constraints.push_back(itcp.constraint);
						sexecution.push_back(duration);
					}
				}
			}

	// write to screen
	printf("x\ty\tz\titerations\tcost\tconstraint\tsexecution\n");
	for (int k = 0; k < costs.size(); k++)
		printf("%f\t%f\t%f\t%d\t%f\t%f\t%ld\n", pts[k].x,
			pts[k].y, pts[k].z, iterations[k], costs[k], constraints[k], sexecution[k]);

	return 0;
}