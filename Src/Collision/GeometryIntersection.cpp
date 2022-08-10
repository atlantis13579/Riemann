
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
inline bool			RayCastT(void* Obj, const Vector3& Origin, const Vector3& Direction, float* t)
{
	T* p = static_cast<T*>(Obj);
	return p->IntersectRay(Origin, Direction, t);
}

bool 				OverlapGJKSolver(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
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

bool				OverlapPlanePlane(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const Plane3d* plane1 = static_cast<const Plane3d*>(Obj1);
	const Plane3d* plane2 = static_cast<const Plane3d*>(Obj2);
	Vector3 Normal = trans->Local1ToLocal2Direction(plane1->Normal);
	Vector3 Origin = trans->Local1ToLocal2(plane1->GetOrigin());
	Plane3d plane_new(Normal, Origin);
	return plane2->IntersectPlane(plane_new.Normal, plane_new.D);
}

inline bool			OverlapBoxBox(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const AxisAlignedBox3d* box1 = static_cast<const AxisAlignedBox3d*>(Obj1);
	const AxisAlignedBox3d* box2 = static_cast<const AxisAlignedBox3d*>(Obj2);
	OrientedBox3d obb1(trans->Local1ToLocal2(box1->GetCenter()), box1->GetExtent(), trans->Local2ToLocal1RotationMatrix());
	OrientedBox3d obb2(Vector3::Zero(), box2->GetExtent(), Matrix3::Identity());
	return obb2.IntersectOBB(obb1);
}

inline bool			OverlapBoxPlane(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const Plane3d* plane = static_cast<const Plane3d*>(Obj2);
	Vector3 Normal = trans->Local2ToLocal1Direction(plane->Normal);
	Vector3 Origin = trans->Local2ToLocal1(plane->GetOrigin());
	Plane3d plane_new(Normal, Origin);
	const AxisAlignedBox3d* box = static_cast<const AxisAlignedBox3d*>(Obj1);
	return plane_new.IntersectAABB(box->Min, box->Max);
}

template <class T>
inline bool			OverlapSphereT(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const Sphere3d* sphere = static_cast<const Sphere3d*>(Obj1);
	float Radius = sphere->Radius;
	Vector3 Center = trans->Local1ToLocal2(sphere->Center);
	const T* p = static_cast<const T*>(Obj2);
	return p->IntersectSphere(Center, Radius);
}

template <class T>
inline bool			OverlapCapsuleT(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const Capsule3d* capsule = static_cast<const Capsule3d*>(Obj1);
	float Radius = capsule->Radius;
	Vector3 P0 = trans->Local1ToLocal2(capsule->X0);
	Vector3 P1 = trans->Local1ToLocal2(capsule->X1);
	const T* p = static_cast<const T*>(Obj2);
	return p->IntersectCapsule(P0, P1, Radius);
}

template <class T>
inline bool			OverlapTTriangle(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const Triangle3d* tri = static_cast<const Triangle3d*>(Obj2);
	Vector3 A = trans->Local2ToLocal1(tri->A);
	Vector3 B = trans->Local2ToLocal1(tri->B);
	Vector3 C = trans->Local2ToLocal1(tri->C);
	const T* p = static_cast<const T*>(Obj1);
	return p->IntersectTriangle(A, B, C);
}

inline bool			OverlapBoxHeightfield(const void* Obj1, const void* Obj2, const Geometry2Transform* trans)
{
	const AxisAlignedBox3d* box1 = static_cast<const AxisAlignedBox3d*>(Obj1);
	const HeightField3d* hf = static_cast<const HeightField3d*>(Obj2);
	(void)box1;
	(void)hf;
	assert(false);
	return false;
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
	REG_GEOMETRY_OBJ(ShapeType3d::CYLINDER,		Cylinder3d)
	REG_GEOMETRY_OBJ(ShapeType3d::CAPSULE,		Capsule3d)
	REG_GEOMETRY_OBJ(ShapeType3d::TRIANGLE,		Triangle3d)
	REG_GEOMETRY_OBJ(ShapeType3d::HEIGHTFIELD,	HeightField3d)
	REG_GEOMETRY_OBJ(ShapeType3d::CONVEX_MESH,	ConvexMesh)
	REG_GEOMETRY_OBJ(ShapeType3d::TRIANGLE_MESH,TriangleMesh)

	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::BOX,				OverlapBoxBox);
	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::PLANE,				OverlapBoxPlane);
	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::TRIANGLE,			OverlapTTriangle<AxisAlignedBox3d>);
	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::HEIGHTFIELD,		OverlapBoxHeightfield);
	REG_OVERLAP_TEST(ShapeType3d::BOX,			ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::PLANE,				OverlapPlanePlane);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::TRIANGLE,			OverlapTTriangle<Plane3d>);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::HEIGHTFIELD,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::PLANE,		ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::BOX,				OverlapSphereT<AxisAlignedBox3d>);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::PLANE,				OverlapSphereT<Plane3d>);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::SPHERE,			OverlapSphereT<Sphere3d>);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::CAPSULE,			OverlapSphereT<Sphere3d>);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::TRIANGLE,			OverlapSphereT<Triangle3d>);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::HEIGHTFIELD,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::SPHERE, 		ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::CYLINDER,		ShapeType3d::BOX,				OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CYLINDER,		ShapeType3d::PLANE,				OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CYLINDER,		ShapeType3d::SPHERE,			OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CYLINDER,		ShapeType3d::CYLINDER,			OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CYLINDER,		ShapeType3d::CAPSULE,			OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CYLINDER,		ShapeType3d::TRIANGLE,			OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CYLINDER,		ShapeType3d::HEIGHTFIELD,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::CYLINDER,		ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::BOX,				OverlapCapsuleT<AxisAlignedBox3d>);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::PLANE,				OverlapCapsuleT<Plane3d>);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::CAPSULE,			OverlapCapsuleT<Capsule3d>);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::TRIANGLE,			OverlapTTriangle<Capsule3d>);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::HEIGHTFIELD,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::CAPSULE, 		ShapeType3d::TRIANGLE_MESH,		nullptr);
	REG_OVERLAP_TEST(ShapeType3d::CONVEX_MESH,	ShapeType3d::BOX,				OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CONVEX_MESH,	ShapeType3d::PLANE,				OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CONVEX_MESH,	ShapeType3d::SPHERE,			OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CONVEX_MESH,	ShapeType3d::CYLINDER,			OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CONVEX_MESH,	ShapeType3d::CAPSULE,			OverlapGJKSolver);
	REG_OVERLAP_TEST(ShapeType3d::CONVEX_MESH,	ShapeType3d::CONVEX_MESH,		OverlapGJKSolver);
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
	return func;
}

SweepFunc GeometryIntersection::GetSweepFunc(ShapeType3d Type1, ShapeType3d Type2)
{
	SweepFunc func = GeometryIntersection::sweepTable[(int)Type1][(int)Type2];
	return func;
}
