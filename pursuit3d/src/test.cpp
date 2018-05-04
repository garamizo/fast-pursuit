#include "Geometry3D.h"
#include "Map.h"
#include <iostream>
#include <cmath>
#include <cfloat>
#include <ctime>

float randb() {
	return (2.0 * (rand() - RAND_MAX/2.0) / RAND_MAX);
}

void test_model() {
	OBB obox0({0.0f, 0.0f, 0.0f}, 
			 {1.0f, 1.0f, 1.0f});

	double th = M_PI / 4.0;
	OBB obox({sqrt(2)+1+0.1, 1.0f, 0.0f}, 
			 {1.0f, 1.0f, 1.0f},
			 {cos(th), -sin(th), 0, sin(th), cos(th), 0, 0, 0, 1});

	// Model model;
	Map map;
	// OBB oboxp[10];
	for(int k = 0; k < 20; k++) {
		float th = randb() * 2;
		OBB *some_box = new OBB({randb() * 10, randb() * 10, randb() * 10}, 
			 {3, 3, 3},
			 {cos(th), -sin(th), 0, sin(th), cos(th), 0, 0, 0, 1});
		map.AddOBB(some_box);
	}
	// map.AddOBB(&obox);
	// map.AddOBB(&obox0);
	// map.AddOBB(&obox);
	
	map.Accelerate({0, 0, 0}, 10);

	std::cout << map << std::endl;

	Ray rays[1000];
	for(int k = 0; k < 1000; k++) {
		rays[k] = Ray({randb() * 10, randb() * 10, randb() * 10},
					  {randb(), randb(), randb()});
	}

	OBB* obox1;

	clock_t begin = clock();
	for(int k = 0; k < 1000; k++) {
		obox1 = map.Raycast(rays[k]);
	}
	clock_t end = clock();

	// std::cout << *map.objects[0] << std::endl;

	// model.SetContent(&obox);
	// Ray ray({-3.5, 0, 0}, {1, 0, 0});
	// ray.NormalizeDirection();

	// Interval xlim = GetInterval(obox, {1, 0, 0});

	if(obox1 != 0)
		std::cout<< "Intersect? " << *obox1 << "\n";
	else
		std::cout << "No intersection" << std::endl;

	std::cout << "Elapsed time: " << (double(end - begin) / CLOCKS_PER_SEC) * 1e6 / 1000 << std::endl;
}


int main() {

	test_model();
	return 0;

	Point v1(5.0f, 3.0f, 2.0f);
	Point v2 = {100.0f, 200.0f, 300.0f};
	Line l1(v1, {10.0f, 2.0f, 3.0f});
	Point right = v1 - v2;
	float d = Distance(v1, v2);
	Sphere sphere({-1, -2, -3}, 0.1f);
	AABB box = FromMinMax(v1, {2, 5, 3});
	// AABB box(v1, {2, 4, 3});
	// AABB box;
	float th = M_PI / 4.0;
	OBB obox(v1, {1, 1, 3}, mat3(cos(th), -sin(th), 0, sin(th), cos(th), 0, 0, 0, 1));
	OBB obox2(v1, {0.1, 0.1, 0.1});
	Ray ray(v1, {-1.0f, 0.0f, 0.0f});
	Plane plane(v1, 0.5);
	float d1 = PlaneEquation(v1, plane);
	// Triangle tri = FromPoints(v1, v2, {10, 2, 1});
	Point p1 = ClosestPoint(sphere, v1);
	Point p2 = ClosestPoint(box, v1);
	Model model;
	// model.SetContent(&obox);
	AABB bounds = model.GetBounds();
	
	std::cout<< "Component 0: " << bounds.size.x << "\n";
	std::cout<< "Component 0: " << bounds.size.y  << "\n";
	std::cout<< "Component 0: " << bounds.size.z << "\n";
	// std::cout<< "Component 0: " << ModelOBB(model, obox2) << "\n";
	
	return 0;
}