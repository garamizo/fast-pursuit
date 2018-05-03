#ifndef _H_GEOMETRY_3D_
#define _H_GEOMETRY_3D_

#include <vector>
#include <utility>
//#include <cfloat>
#include "vectors.h"
#include "matrices.h"
#include "Compare.h"
#include <ostream>

typedef vec3 Point;

typedef struct Line {
   Point start;
   Point end;

   inline Line() {}
   inline Line(const Point& s, const Point& e) :
       start(s), end(e) { }
} Line;

float Length(const Line& line);
float LengthSq(const Line& line);

typedef struct Ray {
   Point origin;
   vec3 direction;
   
   inline Ray() : direction(0.0f, 0.0f, 1.0f) {}
   inline Ray(const Point& o, const vec3& d) :
       origin(o), direction(d) { 
          NormalizeDirection(); 
   }
   inline void NormalizeDirection() {
       Normalize(direction);
   }
} Ray;

Ray FromPoints(const Point& from, const Point& to);

typedef struct Sphere {
   Point position;
   float radius;
   inline Sphere() : radius(1.0f) { }
   inline Sphere(const Point& p, float r) :
       position(p), radius(r) { }
} Sphere;

typedef struct AABB {
   Point position;
   vec3 size;

   inline AABB() : size(1, 1, 1) { }
   inline AABB(const Point& o, const vec3& s) :
       position(o), size(s) { }
} AABB;

vec3 GetMin(const AABB& aabb);
vec3 GetMax(const AABB& aabb);
AABB FromMinMax(const vec3& min, const vec3& max);

typedef struct OBB {
   Point position;
   vec3 size;
   mat3 orientation;
   inline OBB() : size(1, 1, 1) { }
   inline OBB(const Point& p, const vec3& s) :
     position(p), size(s) { }
   inline OBB(const Point& p, const vec3& s, const mat3& o)
       : position(p), size(s), orientation(o) { }
} OBB;

typedef struct Plane {
   vec3 normal;
   float distance;

   inline Plane() : normal(1, 0, 0) { }
   inline Plane(const vec3& n, float d) :
       normal(n), distance(d) { }
} Plane;

float PlaneEquation(const Point& pt, const Plane& plane);

typedef struct Triangle {
   union {
       Point points[3];
       float values[9];
   };
   inline Triangle() { }
} Triangle;

Triangle FromPoints(const Point& a, const Point& b, const Point& c);

bool PointInSphere(const Point& point, 
    const Sphere& sphere);
Point ClosestPoint(const Sphere& sphere, 
    const Point& point);

bool PointInAABB(const Point& point, const AABB& aabb);
Point ClosestPoint(const AABB&aabb, const Point& point);

bool AABBPlane(const AABB& aabb, const Plane& plane);

bool PointInOBB(const Point& point, const OBB& obb);
Point ClosestPoint(const OBB& obb, const Point& point);

#define PlaneAABB(plane, aabb) \
   AABBPlane(aabb, plane)

bool AABBAABB(const AABB& aabb1, const AABB& aabb2);

bool SphereOBB(const Sphere& sphere, const OBB& obb);
#define OBBSphere(obb, sphere) \
SphereOBB(sphere, obb)

bool OverlapOnAxis(const OBB& obb1, const OBB& obb2, 
    const vec3& axis);
bool OBBOBB(const OBB& obb1, const OBB& obb2);

float Raycast(const AABB& aabb, const Ray& ray);
float Raycast(const OBB& obb, const Ray& ray);
float Raycast(const Plane& plane, const Ray& ray);

bool Linetest(const AABB& aabb, const Line& line);
bool Linetest(const OBB& obb, const Line& line);


typedef struct Interval {
   float min;
   float max;
} Interval;

Interval GetInterval(const AABB& rect, const vec3& axis);
Interval GetInterval(const OBB& rect, const vec3& axis);
bool OverlapOnAxis(const AABB& aabb, const OBB& obb, 
   const vec3& axis);
bool AABBOBB(const AABB& aabb, const OBB& obb);
#define OBBAABB(obb, aabb) AABBOBB(aabb, obb)

bool OBBPlane(const OBB&obb, const Plane& plane);
#define PlaneOBB(plane, obb) \
   OBBPlane(obb, plane)

class Model {
protected:
    OBB* content;
    AABB bounds;
public:
    vec3 position;
    vec3 rotation;
    Model* parent;
    inline Model() : parent(0), content(0) { }
    inline OBB* GetMesh() const {
        return content;
    }
    inline AABB GetBounds() const {
        return bounds;
    }
    void SetContent(OBB* mesh);
};

mat4 GetWorldMatrix(const Model& model);
OBB GetOBB(const Model& model);

float ModelRay(const Model& model, const Ray& ray);
bool Linetest(const Model& model, const Line& line);
bool ModelSphere(const Model& model, const Sphere& sphere);
bool ModelAABB(const Model& model, const AABB& aabb);
bool ModelOBB(const Model& model, const OBB& obb);
bool ModelPlane(const Model& model, const Plane& plane);

#endif