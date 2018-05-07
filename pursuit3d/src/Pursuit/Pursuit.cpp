#include "Pursuit.h"

SPT::SPT(Map* _map, const Point& _root) {
	map = _map;
	CreateKeypoints();
	CreateVisibilityGraph();

	dist.resize(pt.size());
	parent.resize(pt.size());
	sptSet.resize(pt.size());

	Rewire(_root);
}

bool SPT::findPath(const Point& dest, PathResult& result) {
	
	int num_nodes = pt.size();
	float dist_min = DIST_MAX;
	int parent_min = -1;

	Line direct(root, dest);
	if (!map->Linetest(direct)) {  // test if direct flight
		dist_min = Length(direct);
		parent_min = num_nodes;

	} else  {// loop through each node and find least path updated cost
		Line line(pt[0], dest);
		for(int i = 0; i < num_nodes; i++) {
			line.start = pt[i];
			float newdist = dist[i] + Length(line);
			if (!map->Linetest(line) && newdist < dist_min) {
				dist_min = newdist;
				parent_min = i;
			}
		}
	}

	if (parent_min > 0) {
		result.dist = dist_min;
		result.waypts.resize(0);
		result.waypts.push_back(dest);
		while(parent_min != num_nodes) {
			result.waypts.push_back(pt[parent_min]);
			parent_min = parent[parent_min];
		}
		result.waypts.push_back(root);
		result.arrive = Normalized(result.waypts[0] - result.waypts[1]);
		std::reverse(std::begin(result.waypts), std::end(result.waypts));
		result.depart = Normalized(result.waypts[1] - result.waypts[0]);

		return true;
	}
	return false;
}

void SPT::printPathResult(const PathResult& result) {

	printf("SPT path (%f):\n", result.dist);
	for(int i = 0; i < result.waypts.size(); i++)
		std::cout << result.waypts[i] << ' ';
	std::cout << '\n';
}

int SPT::minDistance() {
	// Initialize min value
	int min = DIST_MAX, min_index;

	for (int v = 0, size = pt.size(); v < size; v++)
		if (sptSet[v] == false && dist[v] < min) {
			min = dist[v];
			min_index = v;
		}

	return min_index;
}

void SPT::printSolution() {

    printf("Vertex\t Distance\tPath");
    for (int i = 0, size = pt.size(); i < size; i++)
    {
        printf("\n%d \t %f\t\t%d ", i, dist[i], size);
        printPath(i);
    }
    printf("\n");
}

void SPT::printPath(int node) {
     
    // Base Case : If j is source
    if (parent[node] > pt.size())
        return;
 
    printPath(parent[node]);
 
    printf("%d ", node);
}

void SPT::Rewire(const Point& _root) {
	root = _root;

	int V = pt.size();

	// Initialize all distances as INFINITE and stpSet[] as false
	for (int i = 0; i < V; i++) {
		dist[i] = DIST_MAX;
		sptSet[i] = false;
		parent[i] = -1;
	}

     // Distance of source vertex from itself is always 0
    Line line(root, {0, 0, 0});
    for(int i = 0; i < V; i++) {
    	line.end = pt[i];
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
         if (!sptSet[v] && graph[u][v] && dist[u] < DIST_MAX 
                                       && dist[u]+graph[u][v] < dist[v]) {
         	dist[v] = dist[u] + graph[u][v];
         	parent[v] = u;
         }  
     }
  
     // print the constructed distance array
}

void SPT::CreateKeypoints() {
	// create keypoints
	for(int k = 0; k < map->objects.size(); k++) {
		vec3 size = map->objects[k]->size * inflate;
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

		mat4 transf = FromMat3(map->objects[k]->orientation) *
					  Translation(map->objects[k]->position);

		for(int k2 = 0; k2 < 8; k2++) {
			points[k2] = MultiplyPoint(points[k2], transf);
			if(!map->PointInMap(points[k2]) && points[k2].z > 0)
				pt.push_back(points[k2]);
		}

		for(int k2 = 0; k2 < 12; k2++) {
			vec3 dir = Normalized(points[conn2[k2]] - points[conn1[k2]]);
			float mlen = Distance(points[conn1[k2]], points[conn2[k2]]);
			float len_inc = mlen / round(mlen / edge_resolution);
			float clen = len_inc;
			while(clen < mlen * 0.99) {  // avoid floating point error
				Point newpt(points[conn1[k2]] + dir * clen);

				if(!map->PointInMap(newpt) && newpt.z > 0)
				// if(!SimplePointInMap(&map, newpt))
					pt.push_back(newpt);

				clen += len_inc;
			}
		}
	}
}

void SPT::CreateVisibilityGraph() {
	// create visibility map
	int len = pt.size();
	link_count = 0;

	graph.resize(len);
	for(int i = 0; i < len; i++)
		graph[i].resize(len, DIST_MAX);

	for(int i = 0; i < len; i++) {
		for(int j = i+1; j < len; j++) {

			Line line(pt[i], pt[j]);
			if(!map->Linetest(line)) {
				graph[i][j] = graph[j][i] = Length(line);
				link_count++;
			}
		}
	}
}

Planner::Planner(Map* _map) {

	map = _map;
}

bool Planner::AddPursuer(const Point& point, float vel) {
	if(!map->PointInMap(point)) {
		p.push_back(SPT(map, point));
		pvel.push_back(vel);
		return true;
	}
	return false;
}

bool Planner::AddEvader(const Point& point, float vel) {
	if(!map->PointInMap(point)) {
		e.push_back(SPT(map, point));
		evel.push_back(vel);
		return true;
	}
	return false;
}

bool Planner::AddGoal(const Point& point) {
	if(!map->PointInMap(point)) {
		g.push_back(SPT(map, point));
		gvel.push_back(1.0);
		return true;
	}
	return false;
}

bool Planner::SolveInterception(Point point, InterceptionResult& result, std::vector<Point>& sol) {

	sol.resize(0);
	sol.push_back(point);
	Point last_point = point;
	bool valid = true;

	int MAX_ITER = 30;
	for(int i = 0; i < MAX_ITER; i++) {
		valid = EvaluatePoint(point, result);
		if(valid) {
			if (i > 0 && Magnitude(point - last_point) < DIST_MIN)
				return true;

			last_point = point;

			// std::cout << result.constraint << "\t" << result.cost << '\t' << point << '\n';

			vec3 direction = result.costd - Project(result.costd, result.constraintd);
			float mag = 5.0; // 1.0 * (i + MAX_ITER/1.5) * 20.0 / MAX_ITER;
			point = point - result.constraintd * result.constraint
						  - direction * mag;
		}
		else {
			Ray ray(last_point, point - last_point);
			RaycastResult ray_result;
			bool collide = map->Raycast(ray, ray_result);

			if (collide && 
				ray_result.t <= Distance(point, last_point)) {
				// std::cout << '*';
				point = point - Project(point - last_point, ray_result.normal);  
				// point = ray.origin + ray.direction * ray_result.t * 0.9;		
			}
			else  {  // can't recover
				std::cout << "Fail: " << last_point << " -> " << point << "\t"
						  << "t: " << ray_result.t << "\t" << map->PointInMap(last_point) << "\t"
						  << map->PointInMap(point) << "\t" << Distance(point, last_point) << "\n";
				return false;
			}
		}

		sol.push_back(point);
	}
	return valid;
}

bool Planner::EvaluatePoint(const Point& point, InterceptionResult& intercept) {

	float mintime = 1e6;
	PathResult path;
	bool valid = false;
	int idx;

	for(int i = 0; i < p.size(); i++) {
		bool reached = p[i].findPath(point, path);
		if (reached && path.dist / pvel[i] < mintime) {
			mintime = path.dist / pvel[i];
			intercept.p = &p[i];
			intercept.ppath = path;
			idx = i;
			valid = true;
		}
	}
	if(!valid)
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

void Planner::Step() {
	// Update trees

	// Sample valid point on each grid


	// Solve for each initial guess

	// Assign solutions to tracks and compute variance

}
