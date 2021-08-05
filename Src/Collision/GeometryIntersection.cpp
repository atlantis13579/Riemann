
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
inline bool			RayCastT(void* Obj, const Vector3d& Origin, const Vector3d& Dir, float* t)
{
	T* p = static_cast<T*>(Obj);
	return p->IntersectRay(Origin, Dir, t);
}

template <class T>
inline bool			OverlapBoxT(const void* Obj1, const void* Obj2, const Transform &transLocal1ToLocal2)
{
	const AxisAlignedBox3d* box = static_cast<const AxisAlignedBox3d*>(Obj1);
	Vector3d Bmin = transLocal1ToLocal2.LocalToWorldEx(box->Min);
	Vector3d Bmax = transLocal1ToLocal2.LocalToWorldEx(box->Max);
	const T* p = static_cast<const T*>(Obj2);
	return p->IntersectAABB(Bmin, Bmax);
}

template <class T>
inline bool			OverlapTBox(const void* Obj1, const void* Obj2, const Transform& transLocal1ToLocal2)
{
	const T* p = static_cast<const T*>(Obj1);
	const AxisAlignedBox3d* box = static_cast<const AxisAlignedBox3d*>(Obj2);
	Vector3d Bmin = transLocal1ToLocal2.WorldToLocalEx(box->Min);
	Vector3d Bmax = transLocal1ToLocal2.WorldToLocalEx(box->Max);
	return p->IntersectAABB(Bmin, Bmax);
}

template <class T>
inline bool			OverlapSphereT(const void* Obj1, const void* Obj2, const Transform& transLocal1ToLocal2)
{
	const Sphere3d* sphere = static_cast<const Sphere3d*>(Obj2);
	Sphere3d new_sphere;
	new_sphere.Radius = sphere->Radius;
	new_sphere.Center = transLocal1ToLocal2.GetRotation() * sphere->Center + transLocal1ToLocal2.GetTranslation();
	const T* p = static_cast<const T*>(Obj1);
	return p->IntersectSphere(new_sphere);
}

#define	REG_GEOMETRY_OBJ(_type, _name)				\
	raycastTable[_type] = RayCastT<_name>;

#define REG_OVERLAP_TEST(_type1, _type2, _func)		\
	overlapTable[_type1][_type2] = _func;

GeometryIntersection::GeometryIntersection()
{
	REG_GEOMETRY_OBJ(ShapeType::BOX,			AxisAlignedBox3d)
	REG_GEOMETRY_OBJ(ShapeType::PLANE,			Plane3d)
	REG_GEOMETRY_OBJ(ShapeType::SPHERE,			Sphere3d)
	REG_GEOMETRY_OBJ(ShapeType::CAPSULE,		Capsule3d)
	REG_GEOMETRY_OBJ(ShapeType::HEIGHTFIELD,	HeightField3d)
	REG_GEOMETRY_OBJ(ShapeType::CONVEX_MESH,	ConvexMesh)
	REG_GEOMETRY_OBJ(ShapeType::TRIANGLE,		Triangle3d)
	REG_GEOMETRY_OBJ(ShapeType::TRIANGLE_MESH,	TriangleMesh)

	REG_OVERLAP_TEST(ShapeType::BOX, ShapeType::BOX,			OverlapBoxT<AxisAlignedBox3d>);
	REG_OVERLAP_TEST(ShapeType::BOX, ShapeType::PLANE,			OverlapBoxT<Plane3d>);
	REG_OVERLAP_TEST(ShapeType::BOX, ShapeType::SPHERE,			OverlapBoxT<Sphere3d>);
	REG_OVERLAP_TEST(ShapeType::BOX, ShapeType::CAPSULE,		OverlapBoxT<Capsule3d>);
	REG_OVERLAP_TEST(ShapeType::BOX, ShapeType::HEIGHTFIELD,	OverlapBoxT<HeightField3d>);
	REG_OVERLAP_TEST(ShapeType::BOX, ShapeType::TRIANGLE,		OverlapBoxT<Triangle3d>);
	REG_OVERLAP_TEST(ShapeType::BOX, ShapeType::TRIANGLE_MESH,	OverlapBoxT<TriangleMesh>);
	REG_OVERLAP_TEST(ShapeType::PLANE, ShapeType::BOX,			OverlapTBox<Plane3d>);
	REG_OVERLAP_TEST(ShapeType::SPHERE, ShapeType::SPHERE,		OverlapSphereT<Sphere3d>);
	REG_OVERLAP_TEST(ShapeType::SPHERE, ShapeType::BOX,			OverlapTBox<Sphere3d>);
	REG_OVERLAP_TEST(ShapeType::CAPSULE, ShapeType::BOX,		OverlapTBox<Capsule3d>);
	REG_OVERLAP_TEST(ShapeType::HEIGHTFIELD, ShapeType::BOX,	OverlapTBox<HeightField3d>);
	REG_OVERLAP_TEST(ShapeType::TRIANGLE, ShapeType::BOX,		OverlapTBox<Triangle3d>);
	REG_OVERLAP_TEST(ShapeType::TRIANGLE_MESH, ShapeType::BOX,	OverlapTBox<TriangleMesh>);
}

GeometryIntersection s_geom_registration;

RayCastFunc GeometryIntersection::GetRayCastFunc(ShapeType Type)
{
	RayCastFunc func = GeometryIntersection::raycastTable[Type];
#ifdef _DEBUG
	assert(func);
#endif
	return func;
}

OverlapFunc GeometryIntersection::GetOverlapFunc(ShapeType Type1, ShapeType Type2)
{
	OverlapFunc func = GeometryIntersection::overlapTable[Type1][Type2];
#ifdef _DEBUG
	assert(func);
#endif
	return func;
}

SweepFunc GeometryIntersection::GetSweepFunc(ShapeType Type1, ShapeType Type2)
{
	SweepFunc func = GeometryIntersection::sweepTable[Type1][Type2];
#ifdef _DEBUG
	assert(func);
#endif
	return func;
}
