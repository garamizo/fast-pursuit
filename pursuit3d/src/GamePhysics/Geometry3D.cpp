#include "Geometry3D.h"
#include <cmath>
#include <cfloat>
#include <list>

float Length(const Line& line) {
   return Magnitude(line.start - line.end);
}

float LengthSq(const Line& line) {
   return MagnitudeSq(line.start - line.end);
}

Ray FromPoints(const Point& from, const Point& to) {
   return Ray(from, Normalized(to - from));
}

vec3 GetMin(const AABB& aabb) {
   vec3 p1 = aabb.position + aabb.size;
   vec3 p2 = aabb.position - aabb.size;

   return vec3(fminf(p1.x, p2.x), 
               fminf(p1.y, p2.y), 
               fminf(p1.z, p2.z));
}

vec3 GetMax(const AABB& aabb) {
   vec3 p1 = aabb.position + aabb.size;
   vec3 p2 = aabb.position - aabb.size;

   return vec3(fmaxf(p1.x, p2.x), 
               fmaxf(p1.y, p2.y), 
               fmaxf(p1.z, p2.z));
}

AABB FromMinMax(const vec3& min, const vec3& max) {
   return AABB((min + max) * 0.5f, (max - min) * 0.5f);
}

float PlaneEquation(const Point& pt, const Plane& plane){
   return Dot(pt, plane.normal) - plane.distance;
}

Triangle FromPoints(const Point& a, const Point& b, const Point& c) {
	Triangle triangle;
	triangle.points[0] = a;
	triangle.points[1] = b;
	triangle.points[2] = c;
   return triangle;
}

bool PointInSphere(const Point& point, 
   const Sphere& sphere) {
   float magSq = MagnitudeSq(point - sphere.position);
   float radSq = sphere.radius * sphere.radius;
   return magSq < radSq;
}

Point ClosestPoint(const Sphere& sphere, 
   const Point& point) {
   vec3 sphereToPoint = point - sphere.position;
   Normalize(sphereToPoint);
   sphereToPoint = sphereToPoint * sphere.radius;
   return sphereToPoint + sphere.position;
}

bool PointInAABB(const Point& point, const AABB& aabb) {
    Point min = GetMin(aabb);
    Point max = GetMax(aabb);
    if (point.x<min.x || point.y<min.y || point.z<min.z) {
        return false;
    }
    if (point.x>max.x || point.y>max.y || point.z>max.z) {
        return false;
    }

    return true;
}

Point ClosestPoint(const AABB& aabb, const Point& point) {
    Point result = point;
    Point min = GetMin(aabb);
    Point max = GetMax(aabb);
    result.x = (result.x<min.x) ? min.x : result.x;
    result.y = (result.y<min.x) ? min.y : result.y;
    result.z = (result.z<min.x) ? min.z : result.z;
    result.x = (result.x>max.x) ? max.x : result.x;
    result.y = (result.y>max.x) ? max.y : result.y;
    result.z = (result.z>max.x) ? max.z : result.z;

    return result;
}

bool AABBPlane(const AABB& aabb, const Plane& plane) {
    float pLen = aabb.size.x * fabsf(plane.normal.x) +
                 aabb.size.y * fabsf(plane.normal.y) +
                 aabb.size.z * fabsf(plane.normal.z);
    float dot = Dot(plane.normal, aabb.position);
    float dist = dot - plane.distance;
    return fabsf(dist) <= pLen;
}

float Raycast(const AABB& aabb, const Ray& ray) {
   vec3 min = GetMin(aabb);
   vec3 max = GetMax(aabb);
   // NOTE: Any component of direction could be 0!
   // to avoid a division by 0, you need to add 
   // additional safety checks.
   float t1 = (min.x - ray.origin.x) / ray.direction.x;
   float t2 = (max.x - ray.origin.x) / ray.direction.x;
   float t3 = (min.y - ray.origin.y) / ray.direction.y;
   float t4 = (max.y - ray.origin.y) / ray.direction.y;
   float t5 = (min.z - ray.origin.z) / ray.direction.z;
   float t6 = (max.z - ray.origin.z) / ray.direction.z;
   float tmin = fmaxf(
                   fmaxf(
                      fminf(t1, t2), 
                      fminf(t3, t4)
                   ), 
                   fminf(t5, t6)
                );
   float tmax = fminf(
                   fminf(
                      fmaxf(t1, t2), 
                      fmaxf(t3, t4)
                   ), 
                   fmaxf(t5, t6)
                );
   if (tmax< 0) {
       return -1;
   }
   if (tmin>tmax) {
       return -1;
   }
   if (tmin< 0.0f) {
       return tmax;
   }
   return tmin;
}

float Raycast(const OBB& obb, const Ray& ray) {
   const float* o = obb.orientation.asArray;
   const float* size = obb.size.asArray;
   // X, Y and Z axis of OBB
   vec3 X(o[0], o[1], o[2]);
   vec3 Y(o[3], o[4], o[5]);
   vec3 Z(o[6], o[7], o[8]);
   vec3 p = obb.position - ray.origin;
   vec3 f(
       Dot(X, ray.direction),
       Dot(Y, ray.direction),
       Dot(Z, ray.direction)
   );
   vec3 e( 
       Dot(X, p),
       Dot(Y, p),
       Dot(Z, p)
   );
   float t[6] = { 0, 0, 0, 0, 0, 0 };
   for (int i = 0; i < 3; ++i) {
       if (CMP(f[i], 0)) {
           if (-e[i] - size[i]>0 || -e[i] + size[i]<0) {
               return -1;
           }
           f[i] = 0.00001f; // Avoid div by 0!
       }
       t[i * 2 + 0] = (e[i] + size[i]) / f[i]; // min
       t[i * 2 + 1] = (e[i] - size[i]) / f[i]; // max
   }
   float tmin = fmaxf(
                   fmaxf(
                      fminf(t[0], t[1]), 
                      fminf(t[2], t[3])), 
                   fminf(t[4], t[5])
                );
   float tmax = fminf(
                   fminf(
                      fmaxf(t[0], t[1]), 
                      fmaxf(t[2], t[3])), 
                   fmaxf(t[4], t[5])
                );
   if (tmax< 0) {
       return -1.0f;
   }
   if (tmin>tmax) {
       return -1.0f;
   }
   if (tmin< 0.0f) {
       return tmax;
   }

   return tmin;
} 


float Raycast(const Plane& plane, const Ray& ray) {
   float nd = Dot(ray.direction, plane.normal);
   float pn = Dot(ray.origin, plane.normal);
   if (nd>= 0.0f) {
       return -1;
   }

   float t = (plane.distance - pn) / nd;
   if (t >= 0.0f) {
       return t;
   }

   return -1;
}

bool Linetest(const AABB& aabb, const Line& line) {
   Ray ray;
   ray.origin = line.start;
   ray.direction = Normalized(line.end - line.start);
   float t = Raycast(aabb, ray);

   return t >= 0 && t * t <= LengthSq(line);
}

bool Linetest(const OBB& obb, const Line& line) {
   Ray ray;
   ray.origin = line.start;
   ray.direction = Normalized(line.end - line.start);
   float t = Raycast(obb, ray);

   return t >= 0 && t * t <= LengthSq(line);
}

Interval GetInterval(const AABB& aabb, const vec3& axis) {
   vec3 i = GetMin(aabb); 
   vec3 a = GetMax(aabb);

    vec3 vertex[8] = {
        vec3(i.x, a.y, a.z),
        vec3(i.x, a.y, i.z),
        vec3(i.x, i.y, a.z),
        vec3(i.x, i.y, i.z),
        vec3(a.x, a.y, a.z),
        vec3(a.x, a.y, i.z),
        vec3(a.x, i.y, a.z),
        vec3(a.x, i.y, i.z)
    };
    Interval result;
    result.min = result.max = Dot(axis, vertex[0]);

    for (int i = 1; i < 8; ++i) {
        float projection = Dot(axis, vertex[i]);
        result.min = (projection < result.min) ? 
           projection : result.min;
        result.max = (projection > result.max) ? 
           projection : result.max;
    }

    return result;
}

Interval GetInterval(const OBB& obb, const vec3& axis) {
    vec3 vertex[8];
    vec3 C = obb.position;    // OBB Center
        vec3 E = obb.size;    // OBB Extents
    const float* o = obb.orientation.asArray;
    vec3 A[] = {              // OBB Axis
        vec3(o[0], o[1], o[2]),
        vec3(o[3], o[4], o[5]),
        vec3(o[6], o[7], o[8]),
    };
    vertex[0] = C + A[0]*E[0] + A[1]*E[1] + A[2]*E[2];
    vertex[1] = C - A[0]*E[0] + A[1]*E[1] + A[2]*E[2];
    vertex[2] = C + A[0]*E[0] - A[1]*E[1] + A[2]*E[2];
    vertex[3] = C + A[0]*E[0] + A[1]*E[1] - A[2]*E[2];
    vertex[4] = C - A[0]*E[0] - A[1]*E[1] - A[2]*E[2];
    vertex[5] = C + A[0]*E[0] - A[1]*E[1] - A[2]*E[2];
    vertex[6] = C - A[0]*E[0] + A[1]*E[1] - A[2]*E[2];
    vertex[7] = C - A[0]*E[0] - A[1]*E[1] + A[2]*E[2];
    Interval result;
    result.min = result.max = Dot(axis, vertex[0]);

    for (int i = 1; i < 8; ++i) {
        float projection = Dot(axis, vertex[i]);
        result.min = (projection < result.min) ? 
           projection : result.min;
        result.max = (projection > result.max) ? 
           projection : result.max;
    }

    return result;
}

bool OverlapOnAxis(const AABB& aabb, const OBB& obb, 
    const vec3& axis) {
    Interval a = GetInterval(aabb, axis);
    Interval b = GetInterval(obb, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
}

bool AABBOBB(const AABB& aabb, const OBB& obb) {
	const float* o = obb.orientation.asArray;

	vec3 test[15] = {
	   vec3(1, 0, 0),          // AABB axis 1
	   vec3(0, 1, 0),          // AABB axis 2
	   vec3(0, 0, 1),          // AABB axis 3
	   vec3(o[0], o[1], o[2]), // OBB axis 1
	   vec3(o[3], o[4], o[5]), // OBB axis 2
	   vec3(o[6], o[7], o[8])  // OBB axis 3
	   // We will fill out the remaining axis in the next step
	};
  for (int i = 0; i < 3; ++i) { // Fill out rest of axis
    test[6 + i * 3 + 0] = Cross(test[i], test[0]);
    test[6 + i * 3 + 1] = Cross(test[i], test[1]);
    test[6 + i * 3 + 2] = Cross(test[i], test[2]);
  }
  for (int i = 0; i < 15; ++i) {
    if (!OverlapOnAxis(aabb, obb, test[i])) {
      return false; // Seperating axis found
    }
  }

  return true; // Seperating axis not found
}



void Model::SetContent(OBB mesh) {
    // content = mesh;
    // if (content != 0) {
    //     vec3 min = mesh->vertices[0];
    //     vec3 max = mesh->vertices[0];

    //     for (int i = 1; i< mesh->numTriangles * 3; ++i) {
    //         min.x = fminf(mesh->vertices[i].x, min.x);
    //         min.y = fminf(mesh->vertices[i].y, min.y);
    //         min.z = fminf(mesh->vertices[i].z, min.z);
    //         max.x = fmaxf(mesh->vertices[i].x, max.x);
    //         max.y = fmaxf(mesh->vertices[i].y, max.y);
    //         max.z = fmaxf(mesh->vertices[i].z, max.z);
    //     }
    //     bounds = FromMinMax(min, max);
    // }
}

