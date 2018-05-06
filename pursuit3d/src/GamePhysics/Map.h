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
	
	
private:
	Map(const Map&);
	Map& operator=(const Map&);
public:
	OctreeNode* octree;
	std::vector<OBB*> objects;
	inline Map() : octree(0) { } 
	inline ~Map() {
		if (octree != 0) {
			delete octree;
		}
	}

	void AddOBB(OBB* obb);
	void RemoveOBB(OBB* model);

	bool Linetest(const Line& line);
	std::vector<OBB*> Query(const Sphere& sphere);
	std::vector<OBB*> Query(const AABB& aabb);
	OBB* PointInMap(const Point& pt);

	bool Accelerate(const vec3& position, float size);
	friend std::ostream& operator<<(std::ostream& os, const Map& m);
};

void SplitTree(OctreeNode* node, int depth);

void Insert(OctreeNode* node, OBB* obb);
void Remove(OctreeNode* node, OBB* obb);

OBB* FindClosest(const std::vector<OBB*>& set, const Ray& ray);
OBB* PointInOctree(OctreeNode* node, const Point& pt);
std::vector<OBB*> Query(OctreeNode* node, const Sphere& sphere);
std::vector<OBB*> Query(OctreeNode* node, const AABB& aabb);
bool Linetest(OctreeNode* node, const Line& line);

#endif