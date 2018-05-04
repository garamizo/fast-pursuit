#ifndef _H_MAP_
#define _H_MAP_

#include "Geometry3D.h"
#include <vector>

typedef struct OctreeNode {
	AABB bounds;
	OctreeNode* children;
	std::vector<OBB*> obbs;

	inline OctreeNode() : children(0) { }
	inline ~OctreeNode() {
		if (children != 0) {
			delete[] children;
		}
	}
} OctreeNode;

class Map {
protected:
	std::vector<OBB*> objects;
	OctreeNode* octree;
private:
	Map(const Map&);
	Map& operator=(const Map&);
public:
	inline Map() : octree(0) { } 
	inline ~Map() {
		if (octree != 0) {
			delete octree;
		}
	}

	void AddOBB(OBB* obb);
	void RemoveOBB(OBB* model);

	OBB* Raycast(const Ray& ray);
	std::vector<OBB*> Query(const Sphere& sphere);
	std::vector<OBB*> Query(const AABB& aabb);

	bool Accelerate(const vec3& position, float size); 
};

void SplitTree(OctreeNode* node, int depth);

void Insert(OctreeNode* node, OBB* obb);
void Remove(OctreeNode* node, OBB* obb);

OBB* FindClosest(const std::vector<OBB*>& set, const Ray& ray);
OBB* Raycast(OctreeNode* node, const Ray& ray);
std::vector<OBB*> Query(OctreeNode* node, const Sphere& sphere);
std::vector<OBB*> Query(OctreeNode* node, const AABB& aabb);

#endif