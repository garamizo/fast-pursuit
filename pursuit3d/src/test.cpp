#include "Geometry3D.h"
#include <iostream>
#include <cmath>
#include <cfloat>


int main() {
	
	Point v1(5.0f, 3.0f, 2.0f);
	Point v2 = {10.0f, 2.0f, 3.0f};
	Line l1(v1, {10.0f, 2.0f, 3.0f});
	Point right = v1 - v2;
	float d = Distance(v1, v2);
	Sphere sphere({-1, -2, -3}, 0.1f);
	AABB box = FromMinMax(v1, {2, 5, 3});
	// AABB box(v1, {2, 4, 3});
	// AABB box;
	OBB obox(v1, {1, 2, 3});
	Ray ray(v1, {-1.0f, 0.0f, 0.0f});
	Plane plane(v1, 0.5);
	float d1 = PlaneEquation(v1, plane);
	Triangle tri = FromPoints(v1, v2, {10, 2, 1});
	Point p1 = ClosestPoint(sphere, v1);
	Point p2 = ClosestPoint(box, v1);
	
	std::cout<< "Component 0: " << AABBOBB(box, obox) << "\n";
	std::cout<< "Component 0: " << AABBPlane(box, plane) << "\n";
	std::cout<< "Component 0: " << Linetest(obox, l1) << "\n";
	std::cout<< "Component 0: " << Raycast(plane, ray) << "\n";
	
	return 0;
}