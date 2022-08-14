#pragma once

#include "../CollisionPrimitive/ShapeType.h"

struct GeometryTransform;
struct Geometry2Transform;

typedef bool (*RayCastFunc)(void*, const Vector3&, const Vector3&, float*);
typedef bool (*IntersectFunc)(const void*, const void*, const GeometryTransform*, const GeometryTransform*);
typedef bool (*SweepFunc)(const void*, const void*, const GeometryTransform*, const GeometryTransform*, const Vector3&, Vector3*, float*);

class GeometryIntersection
{
public:
	GeometryIntersection();

	static RayCastFunc		GetRayCastFunc(ShapeType3d Type);
	static IntersectFunc	GetIntersectFunc(ShapeType3d Type1, ShapeType3d Type2);
	static SweepFunc		GetSweepFunc(ShapeType3d Type1, ShapeType3d Type2);

	static RayCastFunc		raycastTable[(int)ShapeType3d::TYPE_COUNT];
	static IntersectFunc	intersectTable[(int)ShapeType3d::TYPE_COUNT][(int)ShapeType3d::TYPE_COUNT];
	static SweepFunc		sweepTable[(int)ShapeType3d::TYPE_COUNT][(int)ShapeType3d::TYPE_COUNT];
};
