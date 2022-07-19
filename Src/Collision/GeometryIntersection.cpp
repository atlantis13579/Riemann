
#include "GeometryIntersection.h"
#include "GeometryDifference.h"
#include "GJK.h"

#include "../CollisionPrimitive/AxisAlignedBox3d.h"
#include "../CollisionPrimitive/Plane3d.h"
#include "../CollisionPrimitive/Sphere3d.h"
#include "../CollisionPrimitive/Triangle3d.h"
#include "../CollisionPrimitive/HeightField3d.h"
#include "../CollisionPrimitive/Cylinder3d.h"
#include "../CollisionPrimitive/Capsule3d.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../CollisionPrimitive/TriangleMesh.h"


RayCastFunc		GeometryIntersection::raycastTable[ShapeType3d::GEOMETRY_COUNT] = { 0 };
SweepFunc		GeometryIntersection::sweepTable[ShapeType3d::GEOMETRY_COUNT][ShapeType3d::GEOMETRY_COUNT] = { 0 };
OverlapFunc		GeometryIntersection::overlapTable[ShapeType3d::GEOMETRY_COUNT][ShapeType3d::GEOMETRY_COUNT] = { 0 };

template <class T>
inline bool			RayCastT(void* Obj, const Vector3d& Origin, const Vector3d& Dir, float* t)
{
	T* p = static_cast<T*>(Obj);
	return p->IntersectRay(Origin, Dir, t);
}

bool 				OverlapGJKSolver(const void* Obj1, const void* Obj2, const Transform &transLocal1ToLocal2)
{
	// Hack, see GetShapeObjPtr()
	const Geometry* Geom1 = reinterpret_cast<const Geometry*>((intptr_t)Obj1 - sizeof(Geometry));
	const Geometry* Geom2 = reinterpret_cast<const Geometry*>((intptr_t)Obj2 - sizeof(Geometry));
	
	GeometryDifference shape(Geom1, Geom2);
	Vector3d guess = shape.GetCenter();

	GJKIntersection gjk;
	GJK_result gjk_status = gjk.Solve(&shape, -guess);
	if (gjk_status == GJK_result::Inside)
	{
		return true;
	}
	return false;
}

bool				OverlapPlanePlane(const void* Obj1, const void* Obj2, const Transform& transLocal1ToLocal2)
{
	const Plane3d* plane1 = static_cast<const Plane3d*>(Obj1);
	const Plane3d* plane2 = static_cast<const Plane3d*>(Obj2);
	Vector3d Normal = transLocal1ToLocal2.LocalToWorldDirection(plane1->Normal);
	Vector3d Origin = transLocal1ToLocal2.LocalToWorldEx(plane1->GetOrigin());
	Plane3d plane_new(Normal, Origin, plane1->HalfThickness);
	return plane2->IntersectPlane(plane_new.Normal, plane_new.D);
}

inline bool			OverlapPlaneBox(const void* Obj1, const void* Obj2, const Transform &transLocal1ToLocal2)
{
	const Plane3d* plane = static_cast<const Plane3d*>(Obj1);
	Vector3d Normal = transLocal1ToLocal2.LocalToWorldDirection(plane->Normal);
	Vector3d Origin = transLocal1ToLocal2.LocalToWorldEx(plane->GetOrigin());
	Plane3d plane_new(Normal, Origin, plane->HalfThickness);
	const AxisAlignedBox3d* box = static_cast<const AxisAlignedBox3d*>(Obj2);
	return plane_new.IntersectAABB(box->Min, box->Max);
}

inline bool			OverlapBoxPlane(const void* Obj1, const void* Obj2, const Transform& transLocal1ToLocal2)
{
	const Plane3d* plane = static_cast<const Plane3d*>(Obj2);
	Vector3d Normal = transLocal1ToLocal2.WorldToLocalDirection(plane->Normal);
	Vector3d Origin = transLocal1ToLocal2.WorldToLocalEx(plane->GetOrigin());
	Plane3d plane_new(Normal, Origin, plane->HalfThickness);
	const AxisAlignedBox3d* box = static_cast<const AxisAlignedBox3d*>(Obj1);
	return plane_new.IntersectAABB(box->Min, box->Max);
}

template <class T>
inline bool			OverlapSphereT(const void* Obj1, const void* Obj2, const Transform& transLocal1ToLocal2)
{
	const Sphere3d* sphere = static_cast<const Sphere3d*>(Obj1);
	float Radius = sphere->Radius;
	Vector3d Center = transLocal1ToLocal2.LocalToWorldEx(sphere->Center);
	const T* p = static_cast<const T*>(Obj2);
	return p->IntersectSphere(Center, Radius);
}

template <class T>
inline bool			OverlapTSphere(const void* Obj1, const void* Obj2, const Transform& transLocal1ToLocal2)
{
	const Sphere3d* sphere = static_cast<const Sphere3d*>(Obj2);
	float Radius = sphere->Radius;
	Vector3d Center = transLocal1ToLocal2.WorldToLocalEx(sphere->Center);
	const T* p = static_cast<const T*>(Obj1);
	return p->IntersectSphere(Center, Radius);
}

template <class T>
inline bool			OverlapCapsuleT(const void* Obj1, const void* Obj2, const Transform& transLocal1ToLocal2)
{
	const Capsule3d* capsule = static_cast<const Capsule3d*>(Obj1);
	float Radius = capsule->Radius;
	Vector3d P0 = transLocal1ToLocal2.LocalToWorldEx(capsule->X0);
	Vector3d P1 = transLocal1ToLocal2.LocalToWorldEx(capsule->X1);
	const T* p = static_cast<const T*>(Obj2);
	return p->IntersectCapsule(P0, P1, Radius);
}

template <class T>
inline bool			OverlapTCapsule(const void* Obj1, const void* Obj2, const Transform& transLocal1ToLocal2)
{
	const Capsule3d* capsule = static_cast<const Capsule3d*>(Obj2);
	float Radius = capsule->Radius;
	Vector3d P0 = transLocal1ToLocal2.WorldToLocalEx(capsule->X0);
	Vector3d P1 = transLocal1ToLocal2.WorldToLocalEx(capsule->X1);
	const T* p = static_cast<const T*>(Obj1);
	return p->IntersectCapsule(P0, P1, Radius);
}

template <class T>
inline bool			OverlapTriangleT(const void* Obj1, const void* Obj2, const Transform& transLocal1ToLocal2)
{
	const Triangle3d* tri = static_cast<const Triangle3d*>(Obj1);
	Vector3d A = transLocal1ToLocal2.LocalToWorldEx(tri->A);
	Vector3d B = transLocal1ToLocal2.LocalToWorldEx(tri->B);
	Vector3d C = transLocal1ToLocal2.LocalToWorldEx(tri->C);
	const T* p = static_cast<const T*>(Obj2);
	return p->IntersectTriangle(A, B, C);
}

template <class T>
inline bool			OverlapTTriangle(const void* Obj1, const void* Obj2, const Transform& transLocal1ToLocal2)
{
	const Triangle3d* tri = static_cast<const Triangle3d*>(Obj2);
	Vector3d A = transLocal1ToLocal2.WorldToLocalEx(tri->A);
	Vector3d B = transLocal1ToLocal2.WorldToLocalEx(tri->B);
	Vector3d C = transLocal1ToLocal2.WorldToLocalEx(tri->C);
	const T* p = static_cast<const T*>(Obj1);
	return p->IntersectTriangle(A, B, C);
}

#define	REG_GEOMETRY_OBJ(_type, _name)				\
	raycastTable[_type] = RayCastT<_name>;

#define REG_OVERLAP_TEST(_type1, _type2, _func)		\
	overlapTable[_type1][_type2] = _func;

GeometryIntersection::GeometryIntersection()
{
	REG_GEOMETRY_OBJ(ShapeType3d::BOX,			AxisAlignedBox3d)
	REG_GEOMETRY_OBJ(ShapeType3d::PLANE,		Plane3d)
	REG_GEOMETRY_OBJ(ShapeType3d::SPHERE,		Sphere3d)
	REG_GEOMETRY_OBJ(ShapeType3d::CAPSULE,		Capsule3d)
	REG_GEOMETRY_OBJ(ShapeType3d::TRIANGLE,		Triangle3d)
	REG_GEOMETRY_OBJ(ShapeType3d::HEIGHTFIELD,	HeightField3d)
	REG_GEOMETRY_OBJ(ShapeType3d::CONVEX_MESH,	ConvexMesh)
	REG_GEOMETRY_OBJ(ShapeType3d::TRIANGLE_MESH,TriangleMesh)

	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::BOX,				OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::PLANE,				OverlapBoxPlane);
	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::SPHERE,			OverlapTSphere<AxisAlignedBox3d>);
	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::CAPSULE,			nullptr);
	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::TRIANGLE,			nullptr);
	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::HEIGHTFIELD,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::CONVEX_MESH,		OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::BOX,				OverlapPlaneBox);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::PLANE,				OverlapPlanePlane);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::SPHERE,			OverlapTSphere<Plane3d>);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::CAPSULE,			OverlapTCapsule<Plane3d>);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::TRIANGLE,			OverlapTTriangle<Plane3d>);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::HEIGHTFIELD,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::CONVEX_MESH,		OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::BOX,				OverlapSphereT<AxisAlignedBox3d>);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::PLANE,				OverlapSphereT<Plane3d>);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::SPHERE,			OverlapSphereT<Sphere3d>);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::CAPSULE,			OverlapSphereT<Sphere3d>);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::TRIANGLE,			OverlapSphereT<Triangle3d>);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::HEIGHTFIELD,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::CONVEX_MESH,		OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::BOX,				OverlapCapsuleT<AxisAlignedBox3d>);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::PLANE,				OverlapCapsuleT<Plane3d>);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::SPHERE,			OverlapTSphere<Capsule3d>);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::CAPSULE,			OverlapCapsuleT<Capsule3d>);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::TRIANGLE,			nullptr);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::HEIGHTFIELD,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::CONVEX_MESH,		OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::HEIGHTFIELD, 	ShapeType3d::BOX,				nullptr);
	REG_OVERLAP_TEST(ShapeType3d::CONVEX_MESH, 	ShapeType3d::CONVEX_MESH,		OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::TRIANGLE, 	ShapeType3d::BOX,				nullptr);
	REG_OVERLAP_TEST(ShapeType3d::TRIANGLE, 	ShapeType3d::PLANE,				OverlapTriangleT<Plane3d>);
	REG_OVERLAP_TEST(ShapeType3d::TRIANGLE, 	ShapeType3d::SPHERE,			OverlapTSphere<Triangle3d>);
	REG_OVERLAP_TEST(ShapeType3d::TRIANGLE_MESH, ShapeType3d::BOX,				nullptr);
}

GeometryIntersection s_geom_registration;

RayCastFunc GeometryIntersection::GetRayCastFunc(ShapeType3d Type)
{
	RayCastFunc func = GeometryIntersection::raycastTable[Type];
	assert(func);
	return func;
}

OverlapFunc GeometryIntersection::GetOverlapFunc(ShapeType3d Type1, ShapeType3d Type2)
{
	OverlapFunc func = GeometryIntersection::overlapTable[Type1][Type2];
	assert(func);
	return func;
}

SweepFunc GeometryIntersection::GetSweepFunc(ShapeType3d Type1, ShapeType3d Type2)
{
	SweepFunc func = GeometryIntersection::sweepTable[Type1][Type2];
	assert(func);
	return func;
}
