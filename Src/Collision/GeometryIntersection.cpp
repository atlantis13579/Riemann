
#include "GeometryIntersection.h"
#include "../CollisionPrimitive/AxisAlignedBox3d.h"
#include "../CollisionPrimitive/Plane3d.h"
#include "../CollisionPrimitive/Sphere3d.h"
#include "../CollisionPrimitive/Triangle3d.h"
#include "../CollisionPrimitive/HeightField3d.h"
#include "../CollisionPrimitive/Cylinder3d.h"
#include "../CollisionPrimitive/Capsule3d.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../CollisionPrimitive/TriangleMesh.h"

RayCastFunc		GeometryIntersection::raycastTable[ShapeType::GEOMETRY_COUNT] = { 0 };
SweepFunc		GeometryIntersection::sweepTable[ShapeType::GEOMETRY_COUNT][ShapeType::GEOMETRY_COUNT] = { 0 };
OverlapFunc		GeometryIntersection::overlapTable[ShapeType::GEOMETRY_COUNT][ShapeType::GEOMETRY_COUNT] = { 0 };

template <class T>
static bool			RayCastT(void* Obj, const Vector3d& Origin, const Vector3d& Dir, float* t)
{
	T* p = reinterpret_cast<T*>(Obj);
	return p->IntersectRay(Origin, Dir, t);
}

#define	REG_GEOMETRY_OBJ(_type, _name)									\
	raycastTable[_type] =	RayCastT<_name>;

GeometryIntersection::GeometryIntersection()
{
	REG_GEOMETRY_OBJ(ShapeType::BOX, AxisAlignedBox3d)
	REG_GEOMETRY_OBJ(ShapeType::PLANE, Plane3d)
	REG_GEOMETRY_OBJ(ShapeType::SPHERE, Sphere3d)
	REG_GEOMETRY_OBJ(ShapeType::CAPSULE, Capsule3d)
	REG_GEOMETRY_OBJ(ShapeType::HEIGHTFIELD, HeightField3d)
	REG_GEOMETRY_OBJ(ShapeType::CONVEX_MESH, ConvexMesh)
	REG_GEOMETRY_OBJ(ShapeType::TRIANGLE, Triangle3d)
	REG_GEOMETRY_OBJ(ShapeType::TRIANGLE_MESH, TriangleMesh)
}

GeometryIntersection s_geom_registration;

RayCastFunc GeometryIntersection::GetRayCastFunc(ShapeType Type)
{
	RayCastFunc func = GeometryIntersection::raycastTable[Type];
#ifdef DEBUG
	assert(func);
#endif
	return func;
}

OverlapFunc GeometryIntersection::GetOverlapFunc(ShapeType Type1, ShapeType Type2)
{
	OverlapFunc func = GeometryIntersection::overlapTable[Type1][Type2];
#ifdef DEBUG
	assert(func);
#endif
	return func;
}

SweepFunc GeometryIntersection::GetSweepFunc(ShapeType Type1, ShapeType Type2)
{
	SweepFunc func = GeometryIntersection::sweepTable[Type1][Type2];
#ifdef DEBUG
	assert(func);
#endif
	return func;
}
