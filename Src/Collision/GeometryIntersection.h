#pragma once

#include "../CollisionPrimitive/ShapeType.h"

struct Geometry2Transform;

typedef bool (*RayCastFunc)(void*, const Vector3&, const Vector3&, float*);
typedef bool (*OverlapFunc)(const void*, const void*, const Geometry2Transform*);
typedef bool (*SweepFunc)(const void*, const void*, const Vector3&, float*);

class GeometryIntersection
{
public:
	GeometryIntersection();

	static RayCastFunc		GetRayCastFunc(ShapeType3d Type);
	static OverlapFunc		GetOverlapFunc(ShapeType3d Type1, ShapeType3d Type2);
	static SweepFunc		GetSweepFunc(ShapeType3d Type1, ShapeType3d Type2);

	static RayCastFunc		raycastTable[(int)ShapeType3d::TYPE_COUNT];
	static OverlapFunc		overlapTable[(int)ShapeType3d::TYPE_COUNT][(int)ShapeType3d::TYPE_COUNT];
	static SweepFunc		sweepTable[(int)ShapeType3d::TYPE_COUNT][(int)ShapeType3d::TYPE_COUNT];
};
