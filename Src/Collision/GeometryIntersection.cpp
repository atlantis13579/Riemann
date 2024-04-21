
#include "GeometryObject.h"
#include "GeometryIntersection.h"
#include "GeometryDifference.h"
#include "GJK.h"
#include "EPAPenetration.h"

#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/Plane3.h"
#include "../CollisionPrimitive/Sphere3.h"
#include "../CollisionPrimitive/Triangle3.h"
#include "../CollisionPrimitive/HeightField3.h"
#include "../CollisionPrimitive/Cylinder3.h"
#include "../CollisionPrimitive/OrientedBox3.h"
#include "../CollisionPrimitive/Capsule3.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../CollisionPrimitive/TriangleMesh.h"

namespace Riemann
{
	RayCastFunc		GeometryIntersection::raycastTable[(int)PrimitiveType::TYPE_COUNT] = { 0 };
	IntersectFunc	GeometryIntersection::intersectTable[(int)PrimitiveType::TYPE_COUNT][(int)PrimitiveType::TYPE_COUNT] = { 0 };
	PenetrationFunc	GeometryIntersection::penetrationTable[(int)PrimitiveType::TYPE_COUNT][(int)PrimitiveType::TYPE_COUNT] = { 0 };
	SweepFunc		GeometryIntersection::sweepTable[(int)PrimitiveType::TYPE_COUNT][(int)PrimitiveType::TYPE_COUNT] = { 0 };

	template <class T>
	inline bool			RayCastT(void* Obj, const Vector3& Origin, const Vector3& Direction, float* t)
	{
		T* p = static_cast<T*>(Obj);
		return p->IntersectRay(Origin, Direction, t);
	}

	bool 	IntersectNotSupport(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2)
	{
		return false;
	}

	bool 	IntersectGJKSolver(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2)
	{
		// Hack, see GetShapeObjPtr()
		const GeometryBase* Geom1 = reinterpret_cast<const GeometryBase*>((intptr_t)Obj1 - sizeof(GeometryBase));
		const GeometryBase* Geom2 = reinterpret_cast<const GeometryBase*>((intptr_t)Obj2 - sizeof(GeometryBase));

		GeometryDifference shape(Geom1, Geom2);
		GJKIntersection gjk;
		GJK_status gjk_status = gjk.Solve(&shape);
		if (gjk_status == GJK_status::Intersect)
		{
			return true;
		}
		return false;
	}

	bool	IntersectPlanePlane(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2)
	{
		const GeometryTransform2 trans(t1, t2);
		const Plane3* plane1 = static_cast<const Plane3*>(Obj1);
		const Plane3* plane2 = static_cast<const Plane3*>(Obj2);
		Vector3 Normal = trans.Local1ToLocal2Direction(plane1->Normal);
		Vector3 Origin = trans.Local1ToLocal2(plane1->GetOrigin());
		Plane3 plane_new(Normal, Origin);
		return plane2->IntersectPlane(plane_new.Normal, plane_new.D);
	}

	bool	IntersectBoxBox(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2)
	{
		const AxisAlignedBox3* box1 = static_cast<const AxisAlignedBox3*>(Obj1);
		const AxisAlignedBox3* box2 = static_cast<const AxisAlignedBox3*>(Obj2);
		OrientedBox3 obb1(t1->LocalToWorld(box1->GetCenter()), box1->GetExtent(), t1->transform.quat.ToRotationMatrix3());
		OrientedBox3 obb2(t2->LocalToWorld(box2->GetCenter()), box2->GetExtent(), t2->transform.quat.ToRotationMatrix3());
		return obb1.IntersectOBB(obb2);
	}

	bool	IntersectBoxPlane(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2)
	{
		const GeometryTransform2 trans(t1, t2);
		const Plane3* plane = static_cast<const Plane3*>(Obj2);
		Vector3 Normal = trans.Local2ToLocal1Direction(plane->Normal);
		Vector3 Origin = trans.Local2ToLocal1(plane->GetOrigin());
		Plane3 plane_new(Normal, Origin);
		const AxisAlignedBox3* box = static_cast<const AxisAlignedBox3*>(Obj1);
		return plane_new.IntersectAABB(box->Min, box->Max);
	}

	template <class T>
	bool	IntersectSphereT(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2)
	{
		const GeometryTransform2 trans(t1, t2);
		const Sphere3* sphere = static_cast<const Sphere3*>(Obj1);
		float Radius = sphere->Radius;
		Vector3 Center = trans.Local1ToLocal2(sphere->Center);
		const T* p = static_cast<const T*>(Obj2);
		return p->IntersectSphere(Center, Radius);
	}

	template <class T>
	bool	IntersectCapsuleT(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2)
	{
		const GeometryTransform2 trans(t1, t2);
		const Capsule3* capsule = static_cast<const Capsule3*>(Obj1);
		float Radius = capsule->Radius;
		Vector3 P0 = trans.Local1ToLocal2(capsule->X0);
		Vector3 P1 = trans.Local1ToLocal2(capsule->X1);
		const T* p = static_cast<const T*>(Obj2);
		return p->IntersectCapsule(P0, P1, Radius);
	}

	template <class T>
	bool	IntersectTTriangle(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2)
	{
		const GeometryTransform2 trans(t1, t2);
		const Triangle3* tri = static_cast<const Triangle3*>(Obj2);
		Vector3 A = trans.Local2ToLocal1(tri->v0);
		Vector3 B = trans.Local2ToLocal1(tri->v1);
		Vector3 C = trans.Local2ToLocal1(tri->v2);
		const T* p = static_cast<const T*>(Obj1);
		return p->IntersectTriangle(A, B, C);
	}

	template <class T>
	bool	IntersectBoxT_WS(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2)
	{
		const GeometryTransform2 trans(t1, t2);
		const AxisAlignedBox3* box1 = static_cast<const AxisAlignedBox3*>(Obj1);
		const T* p = static_cast<const T*>(Obj2);
		OrientedBox3 obb1(trans.Local1ToLocal2(box1->GetCenter()), box1->GetExtent(), trans.Local1ToLocal2RotationMatrix());
		return p->IntersectOBB(obb1.Center, obb1.Extent, obb1.Rotation);
	}

	bool	PenetrateEPASolver(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, Vector3* n, float* d)
	{
		// Hack, see GetShapeObjPtr()
		const GeometryBase* Geom1 = reinterpret_cast<const GeometryBase*>((intptr_t)Obj1 - sizeof(GeometryBase));
		const GeometryBase* Geom2 = reinterpret_cast<const GeometryBase*>((intptr_t)Obj2 - sizeof(GeometryBase));

		GeometryDifference shape(Geom1, Geom2);
		GJKIntersection gjk;
		GJK_status gjk_status = gjk.Solve(&shape);
		if (gjk_status != GJK_status::Intersect)
		{
			return false;
		}

		EPAPenetration epa;
		EPA_status epa_result = epa.Solve(gjk.result);
		if (epa_result == EPA_status::Failed || epa_result == EPA_status::FallBack)
		{
			return false;
		}

		*n = epa.penetration_normal;
		*d = epa.penetration_depth;
		return  true;
	}

	bool 	PenetrateNotSupport(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, Vector3* n, float* d)
	{
		return false;
	}

	bool	PenetrateBoxBox(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, Vector3* n, float* d)
	{
		const AxisAlignedBox3* box1 = static_cast<const AxisAlignedBox3*>(Obj1);
		const AxisAlignedBox3* box2 = static_cast<const AxisAlignedBox3*>(Obj2);
		OrientedBox3 obb1(t1->LocalToWorld(box1->GetCenter()), box1->GetExtent(), t1->transform.quat.ToRotationMatrix3());
		OrientedBox3 obb2(t2->LocalToWorld(box2->GetCenter()), box2->GetExtent(), t2->transform.quat.ToRotationMatrix3());
		return obb1.PenetrateOBB(obb2, n, d);
	}

	template <class T>
	bool	PenetrateBoxT_WS(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, Vector3* n, float* d)
	{
		const GeometryTransform2 trans(t1, t2);
		const AxisAlignedBox3* box1 = static_cast<const AxisAlignedBox3*>(Obj1);
		const T* p = static_cast<const T*>(Obj2);
		OrientedBox3 obb1(trans.Local1ToLocal2(box1->GetCenter()), box1->GetExtent(), trans.Local1ToLocal2RotationMatrix());
		return p->PenetrateOBB(obb1.Center, obb1.Extent, obb1.Rotation, n, d);
	}

	template <class T>
	bool	PenetrateSphereT(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, Vector3* n, float* d)
	{
		const GeometryTransform2 trans(t1, t2);
		const Sphere3* sphere = static_cast<const Sphere3*>(Obj1);
		float Radius = sphere->Radius;
		Vector3 Center = trans.Local1ToLocal2(sphere->Center);
		const T* p = static_cast<const T*>(Obj2);
		return p->PenetrateSphere(Center, Radius, n, d);
	}

	template <class T>
	bool	IntersectCapsuleT(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, Vector3* n, float* d)
	{
		const GeometryTransform2 trans(t1, t2);
		const Capsule3* capsule = static_cast<const Capsule3*>(Obj1);
		float Radius = capsule->Radius;
		Vector3 P0 = trans.Local1ToLocal2(capsule->X0);
		Vector3 P1 = trans.Local1ToLocal2(capsule->X1);
		const T* p = static_cast<const T*>(Obj2);
		return p->PenetrateCapsule(P0, P1, Radius, n, d);
	}

	template <class T>
	bool	SweepSphereT(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, const Vector3& Dir, Vector3* n, float* t)
	{
		const GeometryTransform2 trans(t1, t2);
		const Sphere3* sphere = static_cast<const Sphere3*>(Obj1);
		float Radius = sphere->Radius;
		Vector3 Center = trans.Local1ToLocal2(sphere->Center);
		const T* obj = static_cast<const T*>(Obj2);
		return obj->SweepSphere(Dir, Center, Radius, n, t);
	}

#define	REG_RAYCAST_FUNC(_type, _name)					\
	raycastTable[(int)_type] = RayCastT<_name>;

#define REG_INTERSECT_FUNC(_type1, _type2, _func)		\
	intersectTable[(int)_type1][(int)_type2] = _func;

#define REG_PENETRATE_FUNC(_type1, _type2, _func)		\
	penetrationTable[(int)_type1][(int)_type2] = _func;

#define REG_SWEEP_FUNC(_type1, _type2, _func)			\
	sweepTable[(int)_type1][(int)_type2] = _func;

	GeometryIntersection::GeometryIntersection()
	{
		REG_RAYCAST_FUNC(PrimitiveType::BOX, AxisAlignedBox3)
		REG_RAYCAST_FUNC(PrimitiveType::PLANE, Plane3)
		REG_RAYCAST_FUNC(PrimitiveType::SPHERE, Sphere3)
		REG_RAYCAST_FUNC(PrimitiveType::CYLINDER, Cylinder3)
		REG_RAYCAST_FUNC(PrimitiveType::CAPSULE, Capsule3)
		REG_RAYCAST_FUNC(PrimitiveType::HEIGHTFIELD, HeightField3)
		REG_RAYCAST_FUNC(PrimitiveType::CONVEX_MESH, ConvexMesh)
		REG_RAYCAST_FUNC(PrimitiveType::TRIANGLE_MESH, TriangleMesh)

		REG_INTERSECT_FUNC(PrimitiveType::BOX, PrimitiveType::BOX, IntersectBoxBox);
		REG_INTERSECT_FUNC(PrimitiveType::BOX, PrimitiveType::PLANE, IntersectBoxPlane);
		REG_INTERSECT_FUNC(PrimitiveType::BOX, PrimitiveType::HEIGHTFIELD, IntersectBoxT_WS<HeightField3>);
		REG_INTERSECT_FUNC(PrimitiveType::BOX, PrimitiveType::TRIANGLE_MESH, IntersectBoxT_WS<TriangleMesh>);
		REG_INTERSECT_FUNC(PrimitiveType::PLANE, PrimitiveType::PLANE, IntersectPlanePlane);
		REG_INTERSECT_FUNC(PrimitiveType::PLANE, PrimitiveType::HEIGHTFIELD, IntersectNotSupport);
		REG_INTERSECT_FUNC(PrimitiveType::PLANE, PrimitiveType::TRIANGLE_MESH, IntersectNotSupport);
		REG_INTERSECT_FUNC(PrimitiveType::SPHERE, PrimitiveType::BOX, IntersectSphereT<AxisAlignedBox3>);
		REG_INTERSECT_FUNC(PrimitiveType::SPHERE, PrimitiveType::PLANE, IntersectSphereT<Plane3>);
		REG_INTERSECT_FUNC(PrimitiveType::SPHERE, PrimitiveType::SPHERE, IntersectSphereT<Sphere3>);
		REG_INTERSECT_FUNC(PrimitiveType::SPHERE, PrimitiveType::CAPSULE, IntersectSphereT<Sphere3>);
		REG_INTERSECT_FUNC(PrimitiveType::SPHERE, PrimitiveType::HEIGHTFIELD, IntersectSphereT<HeightField3>);
		REG_INTERSECT_FUNC(PrimitiveType::SPHERE, PrimitiveType::TRIANGLE_MESH, IntersectSphereT<TriangleMesh>);
		REG_INTERSECT_FUNC(PrimitiveType::CYLINDER, PrimitiveType::BOX, IntersectGJKSolver);
		REG_INTERSECT_FUNC(PrimitiveType::CYLINDER, PrimitiveType::PLANE, IntersectGJKSolver);
		REG_INTERSECT_FUNC(PrimitiveType::CYLINDER, PrimitiveType::SPHERE, IntersectGJKSolver);
		REG_INTERSECT_FUNC(PrimitiveType::CYLINDER, PrimitiveType::CYLINDER, IntersectGJKSolver);
		REG_INTERSECT_FUNC(PrimitiveType::CYLINDER, PrimitiveType::CAPSULE, IntersectGJKSolver);
		REG_INTERSECT_FUNC(PrimitiveType::CYLINDER, PrimitiveType::HEIGHTFIELD, IntersectNotSupport);
		REG_INTERSECT_FUNC(PrimitiveType::CYLINDER, PrimitiveType::TRIANGLE_MESH, IntersectNotSupport);
		REG_INTERSECT_FUNC(PrimitiveType::CAPSULE, PrimitiveType::BOX, IntersectCapsuleT<AxisAlignedBox3>);
		REG_INTERSECT_FUNC(PrimitiveType::CAPSULE, PrimitiveType::PLANE, IntersectCapsuleT<Plane3>);
		REG_INTERSECT_FUNC(PrimitiveType::CAPSULE, PrimitiveType::CAPSULE, IntersectCapsuleT<Capsule3>);
		REG_INTERSECT_FUNC(PrimitiveType::CAPSULE, PrimitiveType::HEIGHTFIELD, IntersectCapsuleT<HeightField3>);
		REG_INTERSECT_FUNC(PrimitiveType::CAPSULE, PrimitiveType::TRIANGLE_MESH, IntersectCapsuleT<TriangleMesh>);
		REG_INTERSECT_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::BOX, IntersectGJKSolver);
		REG_INTERSECT_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::PLANE, IntersectGJKSolver);
		REG_INTERSECT_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::SPHERE, IntersectGJKSolver);
		REG_INTERSECT_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::CYLINDER, IntersectGJKSolver);
		REG_INTERSECT_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::CAPSULE, IntersectGJKSolver);
		REG_INTERSECT_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::CONVEX_MESH, IntersectGJKSolver);
		REG_INTERSECT_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::HEIGHTFIELD, IntersectNotSupport);
		REG_INTERSECT_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::TRIANGLE_MESH, IntersectNotSupport);
		REG_INTERSECT_FUNC(PrimitiveType::TRIANGLE_MESH, PrimitiveType::TRIANGLE_MESH, IntersectNotSupport);

		REG_PENETRATE_FUNC(PrimitiveType::BOX, PrimitiveType::BOX, PenetrateBoxBox);
		REG_PENETRATE_FUNC(PrimitiveType::BOX, PrimitiveType::SPHERE, PenetrateBoxT_WS<Sphere3>);
		REG_PENETRATE_FUNC(PrimitiveType::BOX, PrimitiveType::PLANE, PenetrateBoxT_WS<Plane3>);
		REG_PENETRATE_FUNC(PrimitiveType::BOX, PrimitiveType::CONVEX_MESH, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::BOX, PrimitiveType::HEIGHTFIELD, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::BOX, PrimitiveType::TRIANGLE_MESH, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::SPHERE, PenetrateSphereT<Sphere3>);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::PLANE, PenetrateSphereT<Plane3>);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::CAPSULE, PenetrateSphereT<Capsule3>);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::CONVEX_MESH, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::HEIGHTFIELD, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::TRIANGLE_MESH, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::CAPSULE, PrimitiveType::CAPSULE, IntersectCapsuleT<Capsule3>);
		REG_PENETRATE_FUNC(PrimitiveType::CAPSULE, PrimitiveType::PLANE, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::CAPSULE, PrimitiveType::CONVEX_MESH, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::CAPSULE, PrimitiveType::HEIGHTFIELD, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::CAPSULE, PrimitiveType::TRIANGLE_MESH, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::BOX, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::PLANE, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::SPHERE, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::CYLINDER, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::CAPSULE, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::HEIGHTFIELD, PenetrateNotSupport);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::TRIANGLE_MESH, PenetrateNotSupport);
		REG_PENETRATE_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::PLANE, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::HEIGHTFIELD, PenetrateNotSupport);
		REG_PENETRATE_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::TRIANGLE_MESH, PenetrateNotSupport);
		REG_PENETRATE_FUNC(PrimitiveType::TRIANGLE_MESH, PrimitiveType::TRIANGLE_MESH, PenetrateNotSupport);

		REG_SWEEP_FUNC(PrimitiveType::SPHERE, PrimitiveType::SPHERE, SweepSphereT<Sphere3>);
	}

	GeometryIntersection s_geom_registration;

	RayCastFunc GeometryIntersection::GetRayCastFunc(PrimitiveType Type)
	{
		RayCastFunc func = GeometryIntersection::raycastTable[(int)Type];
		assert(func);
		return func;
	}

	IntersectFunc GeometryIntersection::GetIntersectFunc(PrimitiveType Type1, PrimitiveType Type2)
	{
		IntersectFunc func = GeometryIntersection::intersectTable[(int)Type1][(int)Type2];
		return func;
	}

	PenetrationFunc GeometryIntersection::GetPenetrationFunc(PrimitiveType Type1, PrimitiveType Type2)
	{
		PenetrationFunc func = GeometryIntersection::penetrationTable[(int)Type1][(int)Type2];
		return func;
	}

	SweepFunc GeometryIntersection::GetSweepFunc(PrimitiveType Type1, PrimitiveType Type2)
	{
		SweepFunc func = GeometryIntersection::sweepTable[(int)Type1][(int)Type2];
		return func;
	}
}