#pragma once

#include "../CollisionPrimitive/ShapeType.h"

namespace Riemann
{
	struct GeometryTransform;
	struct GeometryTransform2;

	typedef bool (*RayCastFunc)(void*, const Vector3&, const Vector3&, float*);
	typedef bool (*IntersectFunc)(const void*, const void*, const GeometryTransform*, const GeometryTransform*);
	typedef bool (*PenetrationFunc)(const void*, const void*, const GeometryTransform*, const GeometryTransform*, Vector3*, float*);
	typedef bool (*SweepFunc)(const void*, const void*, const GeometryTransform*, const GeometryTransform*, const Vector3&, Vector3*, float*);

	class GeometryIntersection
	{
	public:
		GeometryIntersection();

		static RayCastFunc		GetRayCastFunc(ShapeType Type);
		static IntersectFunc	GetIntersectFunc(ShapeType Type1, ShapeType Type2);
		static PenetrationFunc	GetPenetrationFunc(ShapeType Type1, ShapeType Type2);
		static SweepFunc		GetSweepFunc(ShapeType Type1, ShapeType Type2);

		static RayCastFunc		raycastTable[(int)ShapeType::TYPE_COUNT];
		static IntersectFunc	intersectTable[(int)ShapeType::TYPE_COUNT][(int)ShapeType::TYPE_COUNT];
		static PenetrationFunc	penetrationTable[(int)ShapeType::TYPE_COUNT][(int)ShapeType::TYPE_COUNT];
		static SweepFunc		sweepTable[(int)ShapeType::TYPE_COUNT][(int)ShapeType::TYPE_COUNT];
	};
}