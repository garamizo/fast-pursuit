#include <iostream>
#include "vectors.h"
#include <cmath>
#include <cfloat>


int main() {
	
	vec3 v1 = {1.0f, 3.0f, 2.0f};
	vec3 v2 = {1.0f, 2.0f, 3.0f};
	vec3 right = v1 - v2;
	float d = Distance(v1, v2);
	
	std::cout<< "Component 0: " <<right.x<< "\n";
	std::cout<< "Component 0: " <<right.asArray[0] << "\n";
	std::cout<< "Component 0: " <<right[0] << "\n";
	
	return 0;
}