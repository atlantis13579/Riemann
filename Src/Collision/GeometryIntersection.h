#pragma once

#include "../Maths/Vector3d.h"
#include "../CollisionPrimitive/ShapeType.h"

typedef bool			(*RayCastFunc)		(void*, const Vector3d&, const Vector3d&, float*);
typedef bool			(*OverlapFunc)		(const void*, const void*);
typedef bool			(*SweepFunc)		(const void*, const void*, const Vector3d&, float*);

class GeometryIntersection
{
public:
	GeometryIntersection();

	static RayCastFunc		GetRayCastFunc(ShapeType Type);
	static OverlapFunc		GetOverlapFunc(ShapeType Type1, ShapeType Type2);
	static SweepFunc		GetSweepFunc(ShapeType Type1, ShapeType Type2);

	static RayCastFunc		raycastTable[ShapeType::GEOMETRY_COUNT];
	static OverlapFunc		overlapTable[ShapeType::GEOMETRY_COUNT][ShapeType::GEOMETRY_COUNT];
	static SweepFunc		sweepTable[ShapeType::GEOMETRY_COUNT][ShapeType::GEOMETRY_COUNT];
};