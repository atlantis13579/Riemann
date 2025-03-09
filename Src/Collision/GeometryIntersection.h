#pragma once

#include "../CollisionPrimitive/PrimitiveType.h"

namespace Riemann
{
	class Geometry;
	struct GeometryTransform;
	struct GeometryTransform2;

	typedef bool (*RayCastFunc)(void*, const Vector3&, const Vector3&, float*);
	typedef bool (*IntersectFunc)(const void*, const void*, const GeometryTransform*, const GeometryTransform*);
	typedef bool (*PenetrationFunc)(const void*, const void*, const GeometryTransform*, const GeometryTransform*, Vector3*, float*);
	typedef bool (*SweepFunc)(const void*, const void*, const GeometryTransform*, const GeometryTransform*, const Vector3&, Vector3*, Vector3*, float*);

	class GeometryIntersection
	{
	public:
		GeometryIntersection();

		static RayCastFunc		GetRayCastFunc(PrimitiveType Type);
		static IntersectFunc	GetIntersectFunc(PrimitiveType Type1, PrimitiveType Type2);
		static PenetrationFunc	GetPenetrationFunc(PrimitiveType Type1, PrimitiveType Type2);
		static SweepFunc		GetSweepFunc(PrimitiveType Type1, PrimitiveType Type2);

		static RayCastFunc		raycastTable[(int)PrimitiveType::TYPE_COUNT];
		static IntersectFunc	intersectTable[(int)PrimitiveType::TYPE_COUNT][(int)PrimitiveType::TYPE_COUNT];
		static PenetrationFunc	penetrationTable[(int)PrimitiveType::TYPE_COUNT][(int)PrimitiveType::TYPE_COUNT];
		static SweepFunc		sweepTable[(int)PrimitiveType::TYPE_COUNT][(int)PrimitiveType::TYPE_COUNT];
	};

	bool	GJK_Solve(Geometry* Geom1, Geometry* Geom2);
	float	GJK_Solve_Distance(Geometry* Geom1, Geometry* Geom2);
	bool	GJK_Solve_Raycast(const Vector3& Origin, const Vector3& Direction, Geometry* Geom1, float* t);
	bool	GJK_Solve_Shapecast(const Vector3& Origin, const Vector3& Direction, Geometry* castGeom, Geometry* Geom, Vector3* p, Vector3* n, float* t);
}
