
#include "GeometryObject.h"
#include "GeometryIntersection.h"
#include "GeometryDifference.h"
#include "GJK.h"

#include "../CollisionPrimitive/AxisAlignedBox3d.h"
#include "../CollisionPrimitive/Plane3d.h"
#include "../CollisionPrimitive/Sphere3d.h"
#include "../CollisionPrimitive/Triangle3d.h"
#include "../CollisionPrimitive/HeightField3d.h"
#include "../CollisionPrimitive/Cylinder3d.h"
#include "../CollisionPrimitive/OrientedBox3d.h"
#include "../CollisionPrimitive/Capsule3d.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../CollisionPrimitive/TriangleMesh.h"


RayCastFunc		GeometryIntersection::raycastTable[(int)ShapeType3d::TYPE_COUNT] = { 0 };
SweepFunc		GeometryIntersection::sweepTable[(int)ShapeType3d::TYPE_COUNT][(int)ShapeType3d::TYPE_COUNT] = { 0 };
OverlapFunc		GeometryIntersection::overlapTable[(int)ShapeType3d::TYPE_COUNT][(int)ShapeType3d::TYPE_COUNT] = { 0 };

template <class T>
inline bool			RayCastT(void* Obj, const Vector3& Origin, const Vector3& Dir, float* t)
{
	T* p = static_cast<T*>(Obj);
	return p->IntersectRay(Origin, Dir, t);
}

bool 				OverlapGJKSolver(const void* Obj1, const void* Obj2, const Geometry2Transform* transLocal1ToLocal2)
{
	// Hack, see GetShapeObjPtr()
	const Geometry* Geom1 = reinterpret_cast<const Geometry*>((intptr_t)Obj1 - sizeof(Geometry));
	const Geometry* Geom2 = reinterpret_cast<const Geometry*>((intptr_t)Obj2 - sizeof(Geometry));
	
	GeometryDifference shape(Geom1, Geom2);
	GJKIntersection gjk;
	GJK_status gjk_status = gjk.Solve(&shape);
	if (gjk_status == GJK_status::Intersect)
	{
		return true;
	}
	return false;
}

bool				OverlapPlanePlane(const void* Obj1, const void* Obj2, const Geometry2Transform* transLocal1ToLocal2)
{
	const Plane3d* plane1 = static_cast<const Plane3d*>(Obj1);
	const Plane3d* plane2 = static_cast<const Plane3d*>(Obj2);
	Vector3 Normal = transLocal1ToLocal2->Local2ToWorldDirection(plane1->Normal);
	Vector3 Origin = transLocal1ToLocal2->Local2ToWorld(plane1->GetOrigin());
	Plane3d plane_new(Normal, Origin, plane1->HalfThickness);
	return plane2->IntersectPlane(plane_new.Normal, plane_new.D);
}

inline bool			OverlapPlaneBox(const void* Obj1, const void* Obj2, const Geometry2Transform* transLocal1ToLocal2)
{
	const Plane3d* plane = static_cast<const Plane3d*>(Obj1);
	Vector3 Normal = transLocal1ToLocal2->Local2ToWorldDirection(plane->Normal);
	Vector3 Origin = transLocal1ToLocal2->Local2ToWorld(plane->GetOrigin());
	Plane3d plane_new(Normal, Origin, plane->HalfThickness);
	const AxisAlignedBox3d* box = static_cast<const AxisAlignedBox3d*>(Obj2);
	return plane_new.IntersectAABB(box->Min, box->Max);
}

inline bool			OverlapBoxBox(const void* Obj1, const void* Obj2, const Geometry2Transform* transLocal1ToLocal2)
{
	const AxisAlignedBox3d* box1 = static_cast<const AxisAlignedBox3d*>(Obj1);
	const AxisAlignedBox3d* box2 = static_cast<const AxisAlignedBox3d*>(Obj2);
	OrientedBox3d obb1(transLocal1ToLocal2->Local2ToWorld(box1->GetCenter()), box1->GetExtent(), transLocal1ToLocal2->ToRotationMatrix3());
	OrientedBox3d obb2(Vector3::Zero(), box2->GetExtent(), Matrix3::Identity());
	return obb2.IntersectOBB(obb1);
}

inline bool			OverlapBoxPlane(const void* Obj1, const void* Obj2, const Geometry2Transform* transLocal1ToLocal2)
{
	const Plane3d* plane = static_cast<const Plane3d*>(Obj2);
	Vector3 Normal = transLocal1ToLocal2->WorldToLocal2Direction(plane->Normal);
	Vector3 Origin = transLocal1ToLocal2->WorldToLocal2(plane->GetOrigin());
	Plane3d plane_new(Normal, Origin, plane->HalfThickness);
	const AxisAlignedBox3d* box = static_cast<const AxisAlignedBox3d*>(Obj1);
	return plane_new.IntersectAABB(box->Min, box->Max);
}

template <class T>
inline bool			OverlapSphereT(const void* Obj1, const void* Obj2, const Geometry2Transform* transLocal1ToLocal2)
{
	const Sphere3d* sphere = static_cast<const Sphere3d*>(Obj1);
	float Radius = sphere->Radius;
	Vector3 Center = transLocal1ToLocal2->Local2ToWorld(sphere->Center);
	const T* p = static_cast<const T*>(Obj2);
	return p->IntersectSphere(Center, Radius);
}

template <class T>
inline bool			OverlapTSphere(const void* Obj1, const void* Obj2, const Geometry2Transform* transLocal1ToLocal2)
{
	const Sphere3d* sphere = static_cast<const Sphere3d*>(Obj2);
	float Radius = sphere->Radius;
	Vector3 Center = transLocal1ToLocal2->WorldToLocal2(sphere->Center);
	const T* p = static_cast<const T*>(Obj1);
	return p->IntersectSphere(Center, Radius);
}

template <class T>
inline bool			OverlapCapsuleT(const void* Obj1, const void* Obj2, const Geometry2Transform* transLocal1ToLocal2)
{
	const Capsule3d* capsule = static_cast<const Capsule3d*>(Obj1);
	float Radius = capsule->Radius;
	Vector3 P0 = transLocal1ToLocal2->Local2ToWorld(capsule->X0);
	Vector3 P1 = transLocal1ToLocal2->Local2ToWorld(capsule->X1);
	const T* p = static_cast<const T*>(Obj2);
	return p->IntersectCapsule(P0, P1, Radius);
}

template <class T>
inline bool			OverlapTCapsule(const void* Obj1, const void* Obj2, const Geometry2Transform* transLocal1ToLocal2)
{
	const Capsule3d* capsule = static_cast<const Capsule3d*>(Obj2);
	float Radius = capsule->Radius;
	Vector3 P0 = transLocal1ToLocal2->WorldToLocal2(capsule->X0);
	Vector3 P1 = transLocal1ToLocal2->WorldToLocal2(capsule->X1);
	const T* p = static_cast<const T*>(Obj1);
	return p->IntersectCapsule(P0, P1, Radius);
}

template <class T>
inline bool			OverlapTriangleT(const void* Obj1, const void* Obj2, const Geometry2Transform* transLocal1ToLocal2)
{
	const Triangle3d* tri = static_cast<const Triangle3d*>(Obj1);
	Vector3 A = transLocal1ToLocal2->Local2ToWorld(tri->A);
	Vector3 B = transLocal1ToLocal2->Local2ToWorld(tri->B);
	Vector3 C = transLocal1ToLocal2->Local2ToWorld(tri->C);
	const T* p = static_cast<const T*>(Obj2);
	return p->IntersectTriangle(A, B, C);
}

template <class T>
inline bool			OverlapTTriangle(const void* Obj1, const void* Obj2, const Geometry2Transform* transLocal1ToLocal2)
{
	const Triangle3d* tri = static_cast<const Triangle3d*>(Obj2);
	Vector3 A = transLocal1ToLocal2->WorldToLocal2(tri->A);
	Vector3 B = transLocal1ToLocal2->WorldToLocal2(tri->B);
	Vector3 C = transLocal1ToLocal2->WorldToLocal2(tri->C);
	const T* p = static_cast<const T*>(Obj1);
	return p->IntersectTriangle(A, B, C);
}

#define	REG_GEOMETRY_OBJ(_type, _name)				\
	raycastTable[(int)_type] = RayCastT<_name>;

#define REG_OVERLAP_TEST(_type1, _type2, _func)		\
	overlapTable[(int)_type1][(int)_type2] = _func;

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

	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::BOX,				OverlapBoxBox);
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
	RayCastFunc func = GeometryIntersection::raycastTable[(int)Type];
	assert(func);
	return func;
}

OverlapFunc GeometryIntersection::GetOverlapFunc(ShapeType3d Type1, ShapeType3d Type2)
{
	OverlapFunc func = GeometryIntersection::overlapTable[(int)Type1][(int)Type2];
	assert(func);
	return func;
}

SweepFunc GeometryIntersection::GetSweepFunc(ShapeType3d Type1, ShapeType3d Type2)
{
	SweepFunc func = GeometryIntersection::sweepTable[(int)Type1][(int)Type2];
	assert(func);
	return func;
}
