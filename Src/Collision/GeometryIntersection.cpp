
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
IntersectFunc	GeometryIntersection::intersectTable[(int)ShapeType3d::TYPE_COUNT][(int)ShapeType3d::TYPE_COUNT] = { 0 };

template <class T>
inline bool			RayCastT(void* Obj, const Vector3& Origin, const Vector3& Direction, float* t)
{
	T* p = static_cast<T*>(Obj);
	return p->IntersectRay(Origin, Direction, t);
}

bool 				IntersectNotSupport(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	return false;
}

bool 				IntersectGJKSolver(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
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

bool				IntersectPlanePlane(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const Plane3d* plane1 = static_cast<const Plane3d*>(Obj1);
	const Plane3d* plane2 = static_cast<const Plane3d*>(Obj2);
	Vector3 Normal = trans->Local1ToLocal2Direction(plane1->Normal);
	Vector3 Origin = trans->Local1ToLocal2(plane1->GetOrigin());
	Plane3d plane_new(Normal, Origin);
	return plane2->IntersectPlane(plane_new.Normal, plane_new.D);
}

inline bool			IntersectBoxBox(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const AxisAlignedBox3d* box1 = static_cast<const AxisAlignedBox3d*>(Obj1);
	const AxisAlignedBox3d* box2 = static_cast<const AxisAlignedBox3d*>(Obj2);
	OrientedBox3d obb1(trans->Local1ToLocal2(box1->GetCenter()), box1->GetExtent(), trans->Local1ToLocal2RotationMatrix());
	OrientedBox3d obb2(Vector3::Zero(), box2->GetExtent(), Matrix3::Identity());
	return obb2.IntersectOBB(obb1);
}

inline bool			IntersectBoxPlane(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const Plane3d* plane = static_cast<const Plane3d*>(Obj2);
	Vector3 Normal = trans->Local2ToLocal1Direction(plane->Normal);
	Vector3 Origin = trans->Local2ToLocal1(plane->GetOrigin());
	Plane3d plane_new(Normal, Origin);
	const AxisAlignedBox3d* box = static_cast<const AxisAlignedBox3d*>(Obj1);
	return plane_new.IntersectAABB(box->Min, box->Max);
}

template <class T>
inline bool			IntersectSphereT(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const Sphere3d* sphere = static_cast<const Sphere3d*>(Obj1);
	float Radius = sphere->Radius;
	Vector3 Center = trans->Local1ToLocal2(sphere->Center);
	const T* p = static_cast<const T*>(Obj2);
	return p->IntersectSphere(Center, Radius);
}

template <class T>
inline bool			IntersectCapsuleT(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const Capsule3d* capsule = static_cast<const Capsule3d*>(Obj1);
	float Radius = capsule->Radius;
	Vector3 P0 = trans->Local1ToLocal2(capsule->X0);
	Vector3 P1 = trans->Local1ToLocal2(capsule->X1);
	const T* p = static_cast<const T*>(Obj2);
	return p->IntersectCapsule(P0, P1, Radius);
}

template <class T>
inline bool			IntersectTTriangle(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const Triangle3d* tri = static_cast<const Triangle3d*>(Obj2);
	Vector3 A = trans->Local2ToLocal1(tri->A);
	Vector3 B = trans->Local2ToLocal1(tri->B);
	Vector3 C = trans->Local2ToLocal1(tri->C);
	const T* p = static_cast<const T*>(Obj1);
	return p->IntersectTriangle(A, B, C);
}

template <class T>
inline bool			IntersectBoxT_WS(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const AxisAlignedBox3d* box1 = static_cast<const AxisAlignedBox3d*>(Obj1);
	const T* p = static_cast<const T*>(Obj2);
	OrientedBox3d obb1(trans->Local1ToLocal2(box1->GetCenter()), box1->GetExtent(), trans->Local1ToLocal2RotationMatrix());
	return p->IntersectOBB(obb1.Center, obb1.Extent, obb1.Rotation);
}

#define	REG_RAYCAST_FUNC(_type, _name)				\
	raycastTable[(int)_type] = RayCastT<_name>;

#define REG_INTERSECT_FUNC(_type1, _type2, _func)		\
	intersectTable[(int)_type1][(int)_type2] = _func;

GeometryIntersection::GeometryIntersection()
{
	REG_RAYCAST_FUNC(ShapeType3d::BOX,				AxisAlignedBox3d)
	REG_RAYCAST_FUNC(ShapeType3d::PLANE,			Plane3d)
	REG_RAYCAST_FUNC(ShapeType3d::SPHERE,			Sphere3d)
	REG_RAYCAST_FUNC(ShapeType3d::CYLINDER,			Cylinder3d)
	REG_RAYCAST_FUNC(ShapeType3d::CAPSULE,			Capsule3d)
	REG_RAYCAST_FUNC(ShapeType3d::TRIANGLE,			Triangle3d)
	REG_RAYCAST_FUNC(ShapeType3d::HEIGHTFIELD,		HeightField3d)
	REG_RAYCAST_FUNC(ShapeType3d::CONVEX_MESH,		ConvexMesh)
	REG_RAYCAST_FUNC(ShapeType3d::TRIANGLE_MESH,	TriangleMesh)

	REG_INTERSECT_FUNC(ShapeType3d::BOX,			ShapeType3d::BOX,				IntersectBoxBox);
	REG_INTERSECT_FUNC(ShapeType3d::BOX,			ShapeType3d::PLANE,				IntersectBoxPlane);
	REG_INTERSECT_FUNC(ShapeType3d::BOX,			ShapeType3d::TRIANGLE,			IntersectTTriangle<AxisAlignedBox3d>);
	REG_INTERSECT_FUNC(ShapeType3d::BOX,			ShapeType3d::HEIGHTFIELD,		IntersectBoxT_WS<HeightField3d>);
	REG_INTERSECT_FUNC(ShapeType3d::BOX,			ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_INTERSECT_FUNC(ShapeType3d::PLANE,			ShapeType3d::PLANE,				IntersectPlanePlane);
	REG_INTERSECT_FUNC(ShapeType3d::PLANE,			ShapeType3d::TRIANGLE,			IntersectTTriangle<Plane3d>);
	REG_INTERSECT_FUNC(ShapeType3d::PLANE,			ShapeType3d::HEIGHTFIELD,		nullptr);
	REG_INTERSECT_FUNC(ShapeType3d::PLANE,			ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_INTERSECT_FUNC(ShapeType3d::SPHERE, 		ShapeType3d::BOX,				IntersectSphereT<AxisAlignedBox3d>);
	REG_INTERSECT_FUNC(ShapeType3d::SPHERE, 		ShapeType3d::PLANE,				IntersectSphereT<Plane3d>);
	REG_INTERSECT_FUNC(ShapeType3d::SPHERE, 		ShapeType3d::SPHERE,			IntersectSphereT<Sphere3d>);
	REG_INTERSECT_FUNC(ShapeType3d::SPHERE, 		ShapeType3d::CAPSULE,			IntersectSphereT<Sphere3d>);
	REG_INTERSECT_FUNC(ShapeType3d::SPHERE, 		ShapeType3d::TRIANGLE,			IntersectSphereT<Triangle3d>);
	REG_INTERSECT_FUNC(ShapeType3d::SPHERE, 		ShapeType3d::HEIGHTFIELD,		IntersectSphereT<HeightField3d>);
	REG_INTERSECT_FUNC(ShapeType3d::SPHERE, 		ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_INTERSECT_FUNC(ShapeType3d::CYLINDER,		ShapeType3d::BOX,				IntersectGJKSolver);
	REG_INTERSECT_FUNC(ShapeType3d::CYLINDER,		ShapeType3d::PLANE,				IntersectGJKSolver);
	REG_INTERSECT_FUNC(ShapeType3d::CYLINDER,		ShapeType3d::SPHERE,			IntersectGJKSolver);
	REG_INTERSECT_FUNC(ShapeType3d::CYLINDER,		ShapeType3d::CYLINDER,			IntersectGJKSolver);
	REG_INTERSECT_FUNC(ShapeType3d::CYLINDER,		ShapeType3d::CAPSULE,			IntersectGJKSolver);
	REG_INTERSECT_FUNC(ShapeType3d::CYLINDER,		ShapeType3d::TRIANGLE,			IntersectGJKSolver);
	REG_INTERSECT_FUNC(ShapeType3d::CYLINDER,		ShapeType3d::HEIGHTFIELD,		nullptr);
	REG_INTERSECT_FUNC(ShapeType3d::CYLINDER,		ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_INTERSECT_FUNC(ShapeType3d::CAPSULE, 		ShapeType3d::BOX,				IntersectCapsuleT<AxisAlignedBox3d>);
	REG_INTERSECT_FUNC(ShapeType3d::CAPSULE, 		ShapeType3d::PLANE,				IntersectCapsuleT<Plane3d>);
	REG_INTERSECT_FUNC(ShapeType3d::CAPSULE, 		ShapeType3d::CAPSULE,			IntersectCapsuleT<Capsule3d>);
	REG_INTERSECT_FUNC(ShapeType3d::CAPSULE, 		ShapeType3d::TRIANGLE,			IntersectTTriangle<Capsule3d>);
	REG_INTERSECT_FUNC(ShapeType3d::CAPSULE, 		ShapeType3d::HEIGHTFIELD,		IntersectCapsuleT<HeightField3d>);
	REG_INTERSECT_FUNC(ShapeType3d::CAPSULE, 		ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_INTERSECT_FUNC(ShapeType3d::CONVEX_MESH,	ShapeType3d::BOX,				IntersectGJKSolver);
	REG_INTERSECT_FUNC(ShapeType3d::CONVEX_MESH,	ShapeType3d::PLANE,				IntersectGJKSolver);
	REG_INTERSECT_FUNC(ShapeType3d::CONVEX_MESH,	ShapeType3d::SPHERE,			IntersectGJKSolver);
	REG_INTERSECT_FUNC(ShapeType3d::CONVEX_MESH,	ShapeType3d::CYLINDER,			IntersectGJKSolver);
	REG_INTERSECT_FUNC(ShapeType3d::CONVEX_MESH,	ShapeType3d::CAPSULE,			IntersectGJKSolver);
	REG_INTERSECT_FUNC(ShapeType3d::CONVEX_MESH,	ShapeType3d::CONVEX_MESH,		IntersectGJKSolver);
}

GeometryIntersection s_geom_registration;

RayCastFunc GeometryIntersection::GetRayCastFunc(ShapeType3d Type)
{
	RayCastFunc func = GeometryIntersection::raycastTable[(int)Type];
	assert(func);
	return func;
}

IntersectFunc GeometryIntersection::GetIntersectFunc(ShapeType3d Type1, ShapeType3d Type2)
{
	IntersectFunc func = GeometryIntersection::intersectTable[(int)Type1][(int)Type2];
	return func;
}

SweepFunc GeometryIntersection::GetSweepFunc(ShapeType3d Type1, ShapeType3d Type2)
{
	SweepFunc func = GeometryIntersection::sweepTable[(int)Type1][(int)Type2];
	return func;
}
