#include "Pursuit.h"


SPT::SPT(VGraph* _map) {
	map = _map;

	dist.resize(map->pt.size());
	parent.resize(map->pt.size());
	sptSet.resize(map->pt.size());
}


bool SPT::findPath(const Point& dest, PathResult& result) {
	
	int num_nodes = dist.size();
	float dist_min = map->DIST_MAX;
	int parent_min = -1;

	Line direct(root, dest);
	if (!map->Linetest(direct)) {  // test if direct flight
		dist_min = Length(direct);
		parent_min = num_nodes;

	} else  {  // loop through each node and find least path updated cost
		Line line({0, 0, 0}, dest);
		for(int i = 0; i < num_nodes; i++) {
			line.start = map->pt[i];
			float newdist = dist[i] + Length(line);
			if (newdist < dist_min && !map->Linetest(line)) {
				dist_min = newdist;
				parent_min = i;
			}
		}
	}

	if (parent_min == -1)  // no reachable node
		return false;
		
	result.dist = dist_min;
	result.waypts.resize(0);
	result.waypts.push_back(dest);
	while(parent_min != num_nodes) {
		result.waypts.push_back(map->pt[parent_min]);
		parent_min = parent[parent_min];
	}
	result.waypts.push_back(root);
	result.arrive = Normalized(result.waypts[0] - result.waypts[1]);
	std::reverse(std::begin(result.waypts), std::end(result.waypts));
	result.depart = Normalized(result.waypts[1] - result.waypts[0]);

	return true;
}

void SPT::printPathResult(const PathResult& result) {

	printf("SPT path (%f):\n", result.dist);
	for(int i = 0; i < result.waypts.size(); i++)
		std::cout << result.waypts[i] << ' ';
	std::cout << '\n';
}

int SPT::minDistance() {
	// Initialize min value
	int min = map->DIST_MAX, min_index;

	for (int v = 0, size = dist.size(); v < size; v++)
		if (sptSet[v] == false && dist[v] < min) {
			min = dist[v];
			min_index = v;
		}

	return min_index;
}

void SPT::printSolution() {

    printf("Vertex\t Distance\tPath");
    for (int i = 0, size = dist.size(); i < size; i++)
    {
        printf("\n%d \t %f\t\t%d ", i, dist[i], size);
        printPath(i);
    }
    printf("\n");
}

void SPT::printPath(int node) {
     
    // Base Case : If j is source
    if (parent[node] > parent.size())
        return;
 
    printPath(parent[node]);
 
    printf("%d ", node);
}

void SPT::Rewire(const Point& _root) {
	// assume map is setup
	root = _root;

	int V = dist.size();

	// Initialize all distances as INFINITE and stpSet[] as false
	for (int i = 0; i < V; i++) {
		dist[i] = map->DIST_MAX;
		sptSet[i] = false;
		parent[i] = -1;
	}

     // Distance of source vertex from itself is always 0
    Line line(root, {0, 0, 0});
    for(int i = 0; i < V; i++) {
    	line.end = map->pt[i];
    	if(!map->Linetest(line))
    		dist[i] = Length(line), parent[i] = V;
    }
  
     // Find shortest path for all vertices
     for (int count = 0; count < V; count++)
     {
       int u = 	minDistance();
       sptSet[u] = true;
  
       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < V; v++)
         if (!sptSet[v] && map->graph[u][v] && dist[u] < map->DIST_MAX 
                        && dist[u] + map->graph[u][v] < dist[v]) {
         	dist[v] = dist[u] + map->graph[u][v];
         	parent[v] = u;
         }  
     }
  
     // print the constructed distance array
}

void VGraph::Rewire() {
	CreateKeypoints();
	CreateVisibilityGraph();
}

void VGraph::CreateKeypoints() {
	// create keypoints
	pt.resize(0);

	for(int k = 0; k < objects.size(); k++) {
		vec3 size = objects[k].size * inflate;
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

		mat4 transf = FromMat3(objects[k].orientation) *
					  Translation(objects[k].position);

		for(int k2 = 0; k2 < 8; k2++) {
			points[k2] = MultiplyPoint(points[k2], transf);
			if(points[k2].z > 0 && !PointInMap(points[k2]))
				pt.push_back(points[k2]);
		}

		for(int k2 = 0; k2 < 12; k2++) {
			vec3 dir = Normalized(points[conn2[k2]] - points[conn1[k2]]);
			float mlen = Distance(points[conn1[k2]], points[conn2[k2]]);
			float len_inc = mlen / round(mlen / edge_resolution);
			float clen = len_inc;
			while(clen < mlen * 0.99) {  // avoid floating point error
				Point newpt(points[conn1[k2]] + dir * clen);

				if(newpt.z > 0 && !PointInMap(newpt))
					pt.push_back(newpt);

				clen += len_inc;
			}
		}
	}
}

void VGraph::CreateVisibilityGraph() {
	// create visibility map
	int len = pt.size();
	link_count = 0;

	graph.resize(len);
	for(int i = 0; i < len; i++)
		graph[i].resize(len, DIST_MAX);

	for(int i = 0; i < len; i++) {
		for(int j = i+1; j < len; j++) {

			Line line(pt[i], pt[j]);
			if(!Linetest(line)) {
				graph[i][j] = graph[j][i] = Length(line);
				link_count++;
			}
		}
	}
}

Planner::Planner(VGraph* _map) {

	map = _map;
}

Planner::Planner(int nobs, int np, float edge_resolution) {  // random problem

	map = new VGraph();
	map->edge_resolution = edge_resolution;

	Point point;
	vec3 ds = map->bounds.size * 2;
	Point min = GetMin(map->bounds);

	for (int i = 0; i < nobs; i++) {

		point = Point({min.x + rand() * ds.x / RAND_MAX,
				     min.y + rand() * ds.y / RAND_MAX,
				     min.z + rand() * ds.z / RAND_MAX});

		float smin = 0.01f, smax = 1.0f;
		vec3 size({smin + rand() * (smax - smin) / RAND_MAX,
				   smin + rand() * (smax - smin) / RAND_MAX,
				   smin + rand() * (smax - smin) / RAND_MAX});
		float expected_val = (smax + smin) / 2;
		size = size * 0.5 * std::pow(ds.x * ds.y * ds.z / (nobs / 0.1), 1/3.0) / expected_val;

		mat3 rot = Rotation3x3(rand() * 360.0 / RAND_MAX,
							   rand() * 360.0 / RAND_MAX,
							   rand() * 360.0 / RAND_MAX);

		map->AddOBB(OBB(point, size, rot));
	}
	map->Rewire();

	Point min2 = min + ds / 5;
	vec3 ds2 = ds * (3 / 5.0);
	bool valid = false;
	while (!valid) {
		point = Point({min2.x + rand() * ds2.x / RAND_MAX,
				     min2.y + rand() * ds2.y / RAND_MAX,
				     min2.z + rand() * ds2.z / RAND_MAX});
		valid = !map->PointInMap(point);
	}
	AddGoal(point);

	float minspeed = 15.0,
		  speedrange = 5.0;
	int count = 0;
	float speed;
	float mincost = 1e10;
	while (count < np) {

		point = Point({min.x + rand() * ds.x / RAND_MAX,
				     min.y + rand() * ds.y / RAND_MAX,
				     min.z + rand() * ds.z / RAND_MAX});
		speed = minspeed + rand() * speedrange / RAND_MAX;

		if (!map->PointInMap(point)) {
			AddPursuer(point, speed);

			PathResult result;
			if (g[0].findPath(point, result))
				mincost = fminf(mincost, result.dist / speed);

			count++;
		}
	}

	valid = false;
	while (!valid) {
		point = Point({min.x + rand() * ds.x / RAND_MAX,
				     min.y + rand() * ds.y / RAND_MAX,
				     min.z + rand() * ds.z / RAND_MAX});
		speed = minspeed + rand() * speedrange / RAND_MAX;

		PathResult result;
		if (g[0].findPath(point, result))
			valid = result.dist / speed > mincost;
	}
	AddEvader(point, speed);
}


bool Planner::AddPursuer(const Point& point, float vel) {
	if(!map->PointInMap(point)) {
		p.push_back(SPT(map));
		pvel.push_back(vel);
		p[p.size() - 1].Rewire(point);
		return true;
	}
	return false;
}

bool Planner::AddEvader(const Point& point, float vel) {
	if(!map->PointInMap(point)) {
		e.push_back(SPT(map));
		evel.push_back(vel);
		e[e.size() - 1].Rewire(point);
		return true;
	}
	return false;
}

bool Planner::AddGoal(const Point& point) {
	if(!map->PointInMap(point)) {
		g.push_back(SPT(map));
		gvel.push_back(1.0);
		g[g.size() - 1].Rewire(point);
		return true;
	}
	return false;
}

bool Planner::SolveInterceptionAnalysis(Point point, InterceptionResult& outResult, SolverResult* sresult) {

	Point last_point = point;
	int exit_flag = 1, i;
	vec3 update;
	InterceptionResult result;

	for(i = 0; i < MAX_ITER; i++) {

		bool valid = EvaluatePoint(point, result);
		if (sresult != NULL)
			sresult->interception.push_back(result);

		if(valid) {
			last_point = point;

			vec3 cost_action = result.costd * result.cost * cgain; // * (MAX_ITER - i) / MAX_ITER;
			cost_action = cost_action - Project(cost_action, result.constraintd);
			vec3 velocity = update * momentum;
			update = velocity - cost_action
					 - Normalized(result.constraintd) * result.constraint * ggain;
			point = point + update;
			outResult = result;
		}
		else {  // if point is bad
			Ray ray(last_point, point - last_point);
			RaycastResult ray_result;
			bool collide = map->Raycast(ray, &ray_result);

			if (collide && ray_result.t <= Distance(point, last_point)) {
				Point slide_point = point - ray_result.point - 
									Project(point - ray_result.point, ray_result.normal) * 1.01;
				point = ray_result.point + slide_point; 	
			}
			else { // can't recover
				exit_flag = -1; break;
			}
		}

		if (Magnitude(update) < TOL_UPDATE) {
			if (fabs(result.constraint) < TOL_CONSTRAINT)
				exit_flag = 2;
			break;
		}
	}
	if (sresult != NULL) {
		sresult->iterations = i;
		sresult->exit_flag = exit_flag;
	}
	outResult.iterations = i;
	outResult.exit_flag = exit_flag;

	return (exit_flag > 0);
}

bool Planner::SolveInterception(Point point, InterceptionResult& outResult) {

	Point last_point = point;
	int exit_flag = 1, i;
	vec3 update;
	InterceptionResult result;

	for(i = 0; i < MAX_ITER; i++) {

		if(EvaluatePoint(point, result)) {
			last_point = point;

			vec3 cost_action = result.costd * result.cost * cgain; // * (MAX_ITER - i) / MAX_ITER;
			cost_action = cost_action - Project(cost_action, result.constraintd);
			vec3 velocity = update * momentum;
			update = velocity - cost_action
					 - Normalized(result.constraintd) * result.constraint * ggain;
			point = point + update;
			outResult = result;
		}
		else {  // if point is bad
			Ray ray(last_point, point - last_point);
			RaycastResult ray_result;
			bool collide = map->Raycast(ray, &ray_result);

			if (collide && ray_result.t <= Distance(point, last_point)) {
				Point slide_point = point - ray_result.point - 
									Project(point - ray_result.point, ray_result.normal) * 1.01;
				point = ray_result.point + slide_point; 	
			}
			else { // can't recover
				exit_flag = -1; break;
			}
		}

		if (Magnitude(update) < TOL_UPDATE) {
			if (fabs(result.constraint) < TOL_CONSTRAINT)
				exit_flag = 2;
			break;
		}
	}
	outResult.iterations = i;
	outResult.exit_flag = exit_flag;

	return (exit_flag > 0);
}

bool Planner::EvaluatePoint(const Point& point, InterceptionResult& intercept) {

	float mintime = FLT_MAX;
	PathResult path;
	int idx = -1;

	for(int i = 0; i < p.size(); i++) {
		bool reached = p[i].findPath(point, path);
		if (reached && path.dist / pvel[i] < mintime) {
			mintime = path.dist / pvel[i];
			intercept.p = &p[i];
			intercept.ppath = path;
			idx = i;
		}
	}
	if(idx < 0)
		return false;

	if (!e[0].findPath(point, intercept.epath))  // error detection
		return false;
	intercept.e = &e[0];

	if (!g[0].findPath(point, intercept.gpath))  // error detection
		return false;
	intercept.g = &g[0];

	intercept.point = point;
	intercept.cost = intercept.gpath.dist;
	intercept.constraint = intercept.ppath.dist / pvel[idx] - intercept.epath.dist / evel[0];
	intercept.costd = intercept.gpath.arrive;
	intercept.constraintd  = intercept.ppath.arrive / pvel[idx] - intercept.epath.arrive / evel[0];

	return true;
}


std::vector<InterceptionResult> Planner::Step() {
	// Update trees
	vec3 max({70, 70, 50}), min({-70, -30, 1});
	int NDIV = 3;

	vec3 ds({(max.x - min.x) / NDIV, (max.y - min.y) / NDIV, (max.z - min.z) / NDIV});
	std::vector<InterceptionResult> sols;
	int count = 0;
	
	for(int i = 0; i < NDIV; i++)
		for(int j = 0; j < NDIV; j++)
			for(int k = 0; k < NDIV; k++) {

				Point point({min.x + i * ds.x,
						     min.y + j * ds.y,
						     min.z + k * ds.z});

				if (!map->PointInMap(point)) {
					count++;
					InterceptionResult itcp;
					SolverResult sresult;
					if (SolveInterception(point, itcp)) {
						sols.push_back(itcp);
					}
				}
			}

	// Assign solutions to tracks and compute variance
	return(sols);
}


std::ostream& operator<<(std::ostream& os, SPT spt) {
	os << "root: " << spt.root
	   << "\npoint size: " << spt.dist.size() << "\n";
    return os;
}

std::ostream& operator<<(std::ostream& os, Planner planner) {
	os << "cgain: " << planner.cgain
	   << "\nggain: " << planner.ggain
	   << "\nmomentum: " << planner.momentum << "\n";
    return os;
}
