#pragma once

#include "../Maths/Transform.h"
#include "../CollisionPrimitive/ShapeType.h"

typedef bool			(*RayCastFunc)		(void*, const Vector3d&, const Vector3d&, float*);
typedef bool			(*OverlapFunc)		(const void*, const void*, const Transform&);
typedef bool			(*SweepFunc)		(const void*, const void*, const Vector3d&, float*);

class GeometryIntersection
{
public:
	GeometryIntersection();

	static RayCastFunc		GetRayCastFunc(ShapeType3d Type);
	static OverlapFunc		GetOverlapFunc(ShapeType3d Type1, ShapeType3d Type2);
	static SweepFunc		GetSweepFunc(ShapeType3d Type1, ShapeType3d Type2);

	static RayCastFunc		raycastTable[ShapeType3d::GEOMETRY_COUNT];
	static OverlapFunc		overlapTable[ShapeType3d::GEOMETRY_COUNT][ShapeType3d::GEOMETRY_COUNT];
	static SweepFunc		sweepTable[ShapeType3d::GEOMETRY_COUNT][ShapeType3d::GEOMETRY_COUNT];
};
