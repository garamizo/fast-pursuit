#include "Map.h"
#include <algorithm>
#include <list>

void Map::AddOBB(OBB* obb) {
	if (std::find(objects.begin(), objects.end(), obb) != objects.end()) {
		// Duplicate object, don't add
		return;
	}
	objects.push_back(obb);
}

void Map::RemoveOBB(OBB* obb) {
	objects.erase(std::remove(objects.begin(), objects.end(), obb), objects.end());
}



OBB* Map::Raycast(const Ray& ray) {
	if (octree != 0) {
		// :: lets the compiler know to look outside class scope
		return ::Raycast(octree, ray);
	}

	OBB* result = 0;
	float result_t = -1;
	RaycastResult ray_result;

	for (int i = 0, size = objects.size(); i < size; ++i) {
		::Raycast(*objects[i], ray, &ray_result);
		float t = ray_result.t;
		if (result == 0 && t >= 0) {
			result = objects[i];
			result_t = t;
		}
		else if (result != 0 && t < result_t) {
			result = objects[i];
			result_t = t;
		}
	}

	return result;
}

std::vector<OBB*> Map::Query(const Sphere& sphere) {
	if (octree != 0) {
		// :: lets the compiler know to look outside class scope
		return ::Query(octree, sphere);
	}

	std::vector<OBB*> result;
	for (int i = 0, size = objects.size(); i < size; ++i) {
		if (SphereOBB(sphere, *objects[i])) {
			result.push_back(objects[i]);
		}
	}
	return result;
}

std::vector<OBB*> Map::Query(const AABB& aabb) {
	if (octree != 0) {
		// :: lets the compiler know to look outside class scope
		return ::Query(octree, aabb);
	}

	std::vector<OBB*> result;
	for (int i = 0, size = objects.size(); i < size; ++i) {
		if (AABBOBB(aabb, *objects[i])) {
			result.push_back(objects[i]);
		}
	}
	return result;
}

void SplitTree(OctreeNode* node, int depth) {
	if (depth-- <= 0) { // Decrements depth
		return;
	}

	if (node->children == 0) {
		node->children = new OctreeNode[8];

		vec3 c = node->bounds.position;
		vec3 e = node->bounds.size *0.5f;

		node->children[0].bounds = AABB(c + vec3(-e.x, +e.y, -e.z), e);
		node->children[1].bounds = AABB(c + vec3(+e.x, +e.y, -e.z), e);
		node->children[2].bounds = AABB(c + vec3(-e.x, +e.y, +e.z), e);
		node->children[3].bounds = AABB(c + vec3(+e.x, +e.y, +e.z), e);
		node->children[4].bounds = AABB(c + vec3(-e.x, -e.y, -e.z), e);
		node->children[5].bounds = AABB(c + vec3(+e.x, -e.y, -e.z), e);
		node->children[6].bounds = AABB(c + vec3(-e.x, -e.y, +e.z), e);
		node->children[7].bounds = AABB(c + vec3(+e.x, -e.y, +e.z), e);
	}

	if (node->children != 0 && node->obbs.size() > 0) {
		for (int i = 0; i < 8; ++i) { // For each child
			for (int j = 0, size = node->obbs.size(); j < size; ++j) {
				if (AABBOBB(node->children[i].bounds, *node->obbs[j])) {
					node->children[i].obbs.push_back(node->obbs[j]);
				}
			}
		}
		node->obbs.clear();

		// Recurse
		for (int i = 0; i < 8; ++i) {
			SplitTree(&(node->children[i]), depth);
		}
	}
}

void Insert(OctreeNode* node, OBB* obb) {
	if (AABBOBB(node->bounds, *obb)) {
		if (node->children == 0) {
			node->obbs.push_back(obb);
		}
		else {
			for (int i = 0; i < 8; ++i) {
				Insert(&(node->children[i]), obb);
			}
		}
	}
}

void Remove(OctreeNode* node, OBB* obb) {
	if (node->children == 0) {
		std::vector<OBB*>::iterator it = std::find(node->obbs.begin(), node->obbs.end(), obb);
		if (it != node->obbs.end()) {
			node->obbs.erase(it);
		}
	}
	else {
		for (int i = 0; i < 8; ++i) {
			Remove(&(node->children[i]), obb);
		}
	}
}


OBB* FindClosest(const std::vector<OBB*>& set, const Ray& ray) {
	if (set.size() == 0) {
		return 0;
	}

	OBB* closest = 0;
	float closest_t = -1;
	RaycastResult raycast;

	for (int i = 0, size = set.size(); i < size; ++i) {
		// float this_t = OBBRay(*set[i], ray);
		Raycast(*set[i], ray, &raycast);
		float this_t = raycast.t;

		if (this_t < 0) {
			continue;
		}

		if (closest_t < 0 || this_t < closest_t) {
			closest_t = this_t;
			closest = set[i];
		}
	}

	return closest;
}

OBB* Raycast(OctreeNode* node, const Ray& ray) {
	RaycastResult raycast;
	Raycast(node->bounds, ray, &raycast);
	float t = raycast.t;

	if (t >= 0) {
		if (node->children == 0) {
			return FindClosest(node->obbs, ray);
		}
		else {
			std::vector<OBB*> results;
			for (int i = 0; i < 8; ++i) {
				OBB* result = Raycast(&(node->children[i]), ray);
				if (result != 0) {
					results.push_back(result);
				}
			}
			return FindClosest(results, ray);
		}
	}

	return 0;
}

std::vector<OBB*> Query(OctreeNode* node, const Sphere& sphere) {
	std::vector<OBB*> result;

	if (SphereAABB(sphere, node->bounds)) {
		if (node->children == 0) {
			for (int i = 0, size = node->obbs.size(); i < size; ++i) {
				if (SphereOBB(sphere, *(node->obbs[i]))) {
					result.push_back(node->obbs[i]);
				}
			}
		}
		else {
			for (int i = 0; i < 8; ++i) {
				std::vector<OBB*> child = Query(&(node->children[i]), sphere);
				if (child.size() > 0) {
					result.insert(result.end(), child.begin(), child.end());
				}
			}
		}
	}

	return result;
}

std::vector<OBB*> Query(OctreeNode* node, const AABB& aabb) {
	std::vector<OBB*> result;

	if (AABBAABB(aabb, node->bounds)) {
		if (node->children == 0) {
			for (int i = 0, size = node->obbs.size(); i < size; ++i) {
				if (AABBOBB(aabb, *(node->obbs[i]))) {
					result.push_back(node->obbs[i]);
				}
			}
		}
		else {
			for (int i = 0; i < 8; ++i) {
				std::vector<OBB*> child = Query(&(node->children[i]), aabb);
				if (child.size() > 0) {
					result.insert(result.end(), child.begin(), child.end());
				}
			}
		}
	}

	return result;
}

bool Map::Accelerate(const vec3& position, float size) {
	if (octree != 0) {
		return false;
	}

	vec3 min(position.x - size, position.y - size, position.z - size);
	vec3 max(position.x + size, position.y + size, position.z + size);

	// Construct tree root
	octree = new OctreeNode();
	octree->bounds = FromMinMax(min, max);
	octree->children = 0;
	for (int i = 0, size = objects.size(); i < size; ++i) {
		octree->obbs.push_back(objects[i]);
	}

	// Create tree
	SplitTree(octree, 5);
	return true;
}


std::ostream& operator<<(std::ostream& os, const Map& m) {
	os << "Length: " << m.objects.size() << '\n';
	for (int k = 0; k < m.objects.size(); k++)
		os << *m.objects[k] << '\n';
    return os;
}
