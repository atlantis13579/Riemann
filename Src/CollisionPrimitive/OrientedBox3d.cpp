
#include "OrientedBox3d.h"
#include "AxisAlignedBox3d.h"


float OrientedBox3d::SqrDistanceToLine(const Vector3& P0, const Vector3& Dir, float* t) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);

	const Vector3 P0_BoxSpace = (P0 - Center) * Rot;
	const Vector3 Dir_BoxSpace = Dir * Rot;

	float SqrDist = aabb.SqrDistanceToLine(P0_BoxSpace, Dir_BoxSpace, t);
	return SqrDist;
}

float OrientedBox3d::SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);

	const Vector3 P0_BoxSpace = (P0 - Center) * Rot;
	const Vector3 P1_BoxSpace = (P1 - Center) * Rot;
	float SqrDist = aabb.SqrDistanceToSegment(P0_BoxSpace, P1_BoxSpace);
	return SqrDist;
}

float OrientedBox3d::SqrDistanceToPoint(const Vector3& Point) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);

	const Vector3 Point_BoxSpace = (Point - Center) * Rot;

	float SqrDist = aabb.SqrDistanceToPoint(Point_BoxSpace);
	return SqrDist;
}