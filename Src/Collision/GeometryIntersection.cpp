
#include "GeometryObject.h"
#include "GeometryIntersection.h"
#include "GeometryDifference.h"
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
#include "../CollisionPrimitive/GJK.h"

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

    template <class T>
    bool IntersectCapsuleT(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, Vector3* n, float* d)
    {
        const GeometryTransform2 trans(t1, t2);
        const Capsule3* capsule = static_cast<const Capsule3*>(Obj1);
        float Radius = capsule->Radius;
        Vector3 P0 = trans.Local1ToLocal2(capsule->X0);
        Vector3 P1 = trans.Local1ToLocal2(capsule->X1);
        const T* p = static_cast<const T*>(Obj2);
        return p->PenetrateCapsule(P0, P1, Radius, n, d);
    }

	bool	PenetrateEPASolver(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, Vector3* n, float* d)
	{
		// Hack, see GetShapeObjPtr()
		const Geometry* Geom1 = reinterpret_cast<const Geometry*>((intptr_t)Obj1 - sizeof(Geometry));
		const Geometry* Geom2 = reinterpret_cast<const Geometry*>((intptr_t)Obj2 - sizeof(Geometry));

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
	bool	SweepTBox(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, const Vector3& Origin, const Vector3& Dir, Vector3* n, float* t)
	{
        // TODO
        return false;
	}

	template <class T>
	bool	SweepTPlane(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, const Vector3& Origin, const Vector3& Dir, Vector3* n, float* t)
	{
		const GeometryTransform2 trans(t2, t1);
		const Plane3* plane = static_cast<const Plane3*>(Obj2);
		Vector3 OriginNew = trans.Local1ToLocal2(plane->GetOrigin());
		Vector3 NormalNew = trans.Local1ToLocal2Direction(plane->Normal);
		const T* obj = static_cast<const T*>(Obj1);
		Plane3 NewPlane(NormalNew, OriginNew);
		return obj->SweepPlane(Origin, Dir, NewPlane.Normal, NewPlane.D, n, t);
	}

	template <class T>
	bool	SweepTSphere(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, const Vector3& Origin, const Vector3& Dir, Vector3* n, float* t)
	{
		const GeometryTransform2 trans(t2, t1);
		const Sphere3* sphere = static_cast<const Sphere3*>(Obj2);
		float Radius = sphere->Radius;
		Vector3 Center = trans.Local1ToLocal2(sphere->Center);
		const T* obj = static_cast<const T*>(Obj1);
		return obj->SweepSphere(Origin, Dir, Center, Radius, n, t);
	}

    template <class T>
    bool    SweepTCylinder(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, const Vector3& Origin, const Vector3& Dir, Vector3* n, float* t)
    {
        const GeometryTransform2 trans(t2, t1);
        const Cylinder3* cy = static_cast<const Cylinder3*>(Obj2);
        float Radius = cy->Radius;
        Vector3 P0 = trans.Local1ToLocal2(cy->X0);
        Vector3 P1 = trans.Local1ToLocal2(cy->X1);
        const T* obj = static_cast<const T*>(Obj1);
        return obj->SweepCylinder(Origin, Dir, P0, P1, Radius, n, t);
    }

    template <class T>
    bool    SweepTCapsule(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, const Vector3& Origin, const Vector3& Dir, Vector3* n, float* t)
    {
        const GeometryTransform2 trans(t2, t1);
        const Capsule3* capsule = static_cast<const Capsule3*>(Obj2);
        float Radius = capsule->Radius;
        Vector3 P0 = trans.Local1ToLocal2(capsule->X0);
        Vector3 P1 = trans.Local1ToLocal2(capsule->X1);
        const T* obj = static_cast<const T*>(Obj1);
        return obj->SweepCapsule(Origin, Dir, P0, P1, Radius, n, t);
        return false;
    }

    template <class T>
    bool    SweepTConvexmesh(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, const Vector3& Origin, const Vector3& Dir, Vector3* n, float* t)
    {
        const GeometryTransform2 trans(t2, t1);
        const ConvexMesh* convex = static_cast<const ConvexMesh*>(Obj2);
        const T* obj = static_cast<const T*>(Obj1);
        return obj->SweepConvex(Origin, Dir, convex, n, t);
    }

    template <class T>
    bool    SweepTHeightfield(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, const Vector3& Origin, const Vector3& Dir, Vector3* n, float* t)
    {
        const GeometryTransform2 trans(t2, t1);
        const HeightField3* hf = static_cast<const HeightField3*>(Obj2);
        const T* obj = static_cast<const T*>(Obj1);
        return obj->SweepHeightField(Origin, Dir, hf, n, t);
    }

    template <class T>
    bool    SweepTTriangleMesh(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, const Vector3& Origin, const Vector3& Dir, Vector3* n, float* t)
    {
        const GeometryTransform2 trans(t2, t1);
        const TriangleMesh* trimesh = static_cast<const TriangleMesh*>(Obj2);
        const T* obj = static_cast<const T*>(Obj1);
        return obj->SweepTriangleMesh(Origin, Dir, trimesh, n, t);
    }

    bool SweepNotSuppoer(const void* Obj1, const void* Obj2, const GeometryTransform* t1, const GeometryTransform* t2, const Vector3& Origin, const Vector3& Dir, Vector3* n, float* t)
    {
        return true;
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
		REG_PENETRATE_FUNC(PrimitiveType::BOX, PrimitiveType::PLANE, PenetrateBoxT_WS<Plane3>);
		REG_PENETRATE_FUNC(PrimitiveType::BOX, PrimitiveType::SPHERE, PenetrateBoxT_WS<Sphere3>);
		REG_PENETRATE_FUNC(PrimitiveType::BOX, PrimitiveType::CONVEX_MESH, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::BOX, PrimitiveType::HEIGHTFIELD, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::BOX, PrimitiveType::TRIANGLE_MESH, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::PLANE, PenetrateSphereT<Plane3>);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::SPHERE, PenetrateSphereT<Sphere3>);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::CAPSULE, PenetrateSphereT<Capsule3>);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::CONVEX_MESH, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::HEIGHTFIELD, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::SPHERE, PrimitiveType::TRIANGLE_MESH, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::CAPSULE, PrimitiveType::PLANE, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::CAPSULE, PrimitiveType::CAPSULE, IntersectCapsuleT<Capsule3>);
		REG_PENETRATE_FUNC(PrimitiveType::CAPSULE, PrimitiveType::CONVEX_MESH, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CAPSULE, PrimitiveType::HEIGHTFIELD, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::CAPSULE, PrimitiveType::TRIANGLE_MESH, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::PLANE, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::BOX, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::SPHERE, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::CYLINDER, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::CAPSULE, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::CONVEX_MESH, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::HEIGHTFIELD, PenetrateNotSupport);
		REG_PENETRATE_FUNC(PrimitiveType::CYLINDER, PrimitiveType::TRIANGLE_MESH, PenetrateNotSupport);
		REG_PENETRATE_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::PLANE, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::CONVEX_MESH, PenetrateEPASolver);
		REG_PENETRATE_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::HEIGHTFIELD, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::TRIANGLE_MESH, nullptr);
		REG_PENETRATE_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::HEIGHTFIELD, PenetrateNotSupport);
		REG_PENETRATE_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::TRIANGLE_MESH, PenetrateNotSupport);
		REG_PENETRATE_FUNC(PrimitiveType::TRIANGLE_MESH, PrimitiveType::TRIANGLE_MESH, PenetrateNotSupport);

        REG_SWEEP_FUNC(PrimitiveType::BOX, PrimitiveType::BOX, SweepTBox<AxisAlignedBox3>);
        REG_SWEEP_FUNC(PrimitiveType::BOX, PrimitiveType::PLANE, SweepTPlane<AxisAlignedBox3>);
        REG_SWEEP_FUNC(PrimitiveType::BOX, PrimitiveType::SPHERE, SweepTSphere<AxisAlignedBox3>);
        REG_SWEEP_FUNC(PrimitiveType::BOX, PrimitiveType::CYLINDER, SweepTCylinder<AxisAlignedBox3>);
        REG_SWEEP_FUNC(PrimitiveType::BOX, PrimitiveType::CAPSULE, SweepTCapsule<AxisAlignedBox3>);
        REG_SWEEP_FUNC(PrimitiveType::BOX, PrimitiveType::HEIGHTFIELD, SweepTHeightfield<AxisAlignedBox3>);
        REG_SWEEP_FUNC(PrimitiveType::BOX, PrimitiveType::CONVEX_MESH, SweepTConvexmesh<AxisAlignedBox3>);
        REG_SWEEP_FUNC(PrimitiveType::BOX, PrimitiveType::TRIANGLE_MESH, SweepTTriangleMesh<AxisAlignedBox3>);
        REG_SWEEP_FUNC(PrimitiveType::SPHERE, PrimitiveType::BOX, SweepTBox<Sphere3>);
        REG_SWEEP_FUNC(PrimitiveType::SPHERE, PrimitiveType::PLANE, SweepTPlane<Sphere3>);
        REG_SWEEP_FUNC(PrimitiveType::SPHERE, PrimitiveType::SPHERE, SweepTSphere<Sphere3>);
        REG_SWEEP_FUNC(PrimitiveType::SPHERE, PrimitiveType::CYLINDER, SweepTCylinder<Sphere3>);
        REG_SWEEP_FUNC(PrimitiveType::SPHERE, PrimitiveType::CAPSULE, SweepTCapsule<Sphere3>);
        REG_SWEEP_FUNC(PrimitiveType::SPHERE, PrimitiveType::HEIGHTFIELD, SweepTHeightfield<Sphere3>);
        REG_SWEEP_FUNC(PrimitiveType::SPHERE, PrimitiveType::CONVEX_MESH, SweepTConvexmesh<Sphere3>);
        REG_SWEEP_FUNC(PrimitiveType::SPHERE, PrimitiveType::TRIANGLE_MESH, SweepTTriangleMesh<Sphere3>);
        REG_SWEEP_FUNC(PrimitiveType::CYLINDER, PrimitiveType::BOX, SweepTBox<Cylinder3>);
        REG_SWEEP_FUNC(PrimitiveType::CYLINDER, PrimitiveType::PLANE, SweepTPlane<Cylinder3>);
        REG_SWEEP_FUNC(PrimitiveType::CYLINDER, PrimitiveType::SPHERE, SweepTSphere<Cylinder3>);
        REG_SWEEP_FUNC(PrimitiveType::CYLINDER, PrimitiveType::CYLINDER, SweepTCylinder<Cylinder3>);
        REG_SWEEP_FUNC(PrimitiveType::CYLINDER, PrimitiveType::CAPSULE, SweepTCapsule<Cylinder3>);
        REG_SWEEP_FUNC(PrimitiveType::CYLINDER, PrimitiveType::HEIGHTFIELD, SweepTHeightfield<Cylinder3>);
        REG_SWEEP_FUNC(PrimitiveType::CYLINDER, PrimitiveType::CONVEX_MESH, SweepTConvexmesh<Cylinder3>);
        REG_SWEEP_FUNC(PrimitiveType::CYLINDER, PrimitiveType::TRIANGLE_MESH, SweepTTriangleMesh<Cylinder3>);
        REG_SWEEP_FUNC(PrimitiveType::CAPSULE, PrimitiveType::BOX, SweepTBox<Capsule3>);
		REG_SWEEP_FUNC(PrimitiveType::CAPSULE, PrimitiveType::PLANE, SweepTPlane<Capsule3>);
		REG_SWEEP_FUNC(PrimitiveType::CAPSULE, PrimitiveType::SPHERE, SweepTSphere<Capsule3>);
		REG_SWEEP_FUNC(PrimitiveType::CAPSULE, PrimitiveType::CYLINDER, SweepTCylinder<Capsule3>);
        REG_SWEEP_FUNC(PrimitiveType::CAPSULE, PrimitiveType::CAPSULE, SweepTCapsule<Capsule3>);
        REG_SWEEP_FUNC(PrimitiveType::CAPSULE, PrimitiveType::HEIGHTFIELD, SweepTHeightfield<Capsule3>);
        REG_SWEEP_FUNC(PrimitiveType::CAPSULE, PrimitiveType::CONVEX_MESH, SweepTConvexmesh<Capsule3>);
        REG_SWEEP_FUNC(PrimitiveType::CAPSULE, PrimitiveType::TRIANGLE_MESH, SweepTTriangleMesh<Capsule3>);
        REG_SWEEP_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::BOX, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::PLANE, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::SPHERE, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::CYLINDER, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::CAPSULE, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::HEIGHTFIELD, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::CONVEX_MESH, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::HEIGHTFIELD, PrimitiveType::TRIANGLE_MESH, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::BOX, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::PLANE, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::SPHERE, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::CYLINDER, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::CAPSULE, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::HEIGHTFIELD, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::CONVEX_MESH, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::CONVEX_MESH, PrimitiveType::TRIANGLE_MESH, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::TRIANGLE_MESH, PrimitiveType::BOX, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::TRIANGLE_MESH, PrimitiveType::PLANE, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::TRIANGLE_MESH, PrimitiveType::SPHERE, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::TRIANGLE_MESH, PrimitiveType::CYLINDER, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::TRIANGLE_MESH, PrimitiveType::CAPSULE, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::TRIANGLE_MESH, PrimitiveType::HEIGHTFIELD, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::TRIANGLE_MESH, PrimitiveType::CONVEX_MESH, SweepNotSuppoer);
        REG_SWEEP_FUNC(PrimitiveType::TRIANGLE_MESH, PrimitiveType::TRIANGLE_MESH, SweepNotSuppoer);
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

	bool GJK_Solve(Geometry* Geom1, Geometry* Geom2)
	{
		GeometryDifference shape(Geom1, Geom2);
		GJKIntersection gjk;
		GJK_status gjk_status = gjk.Solve(&shape);
		if (gjk_status == GJK_status::Intersect)
		{
			return true;
		}
		return false;
	}

	float GJK_Solve_Distance(Geometry* Geom1, Geometry* Geom2)
	{
		GeometryDifference shape(Geom1, Geom2);
		GJKClosestDistance gjk;
		bool distance = gjk.Solve(&shape);
		return distance;
	}

	bool GJK_Solve_Raycast(const Vector3& Origin, const Vector3& Direction, Geometry* Geom1, float* t)
	{
		TransformedGeometry shape(Geom1);
		GJKRaycast gjk;
		GJK_status gjk_status = gjk.Solve(Origin, Direction, &shape);
		if (gjk_status == GJK_status::Intersect)
		{
			*t = gjk.time;
			return true;
		}
		return false;
	}

	bool GJK_Solve_Shapecast(const Vector3& Origin, const Vector3& Direction, Geometry* castGeom, Geometry* Geom, Vector3* n, float* t)
	{
		TransformedGeometry cast_shape(castGeom);
		TransformedGeometry shape(Geom);
		GJKShapecast gjk;
		bool success = gjk.Solve(Origin, Direction, &cast_shape, &shape, n, t);
		if (success)
		{
			// TODO, local to world
			return true;
		}
		return false;
	}
}
