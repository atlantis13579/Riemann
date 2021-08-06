
#pragma once

#include "ShapeType.h"
#include "../Maths/Vector3d.h"
#include "../Maths/Matrix3d.h"

class OrientedBox3d
{
public:
	Vector3d	Center;
	Vector3d	Extent;
	Matrix3d	Rot;

	OrientedBox3d()
	{
	}

	OrientedBox3d(const Vector3d& _Center, const Vector3d& _Extent, const Matrix3d& _Rot)
	{
		Center = _Center;
		Extent = _Extent;
		Rot = _Rot;
	}

	float SqrDistanceToPoint(const Vector3d& Point) const;
	float SqrDistanceToLine(const Vector3d& P0, const Vector3d& Dir, float* t) const;
	float SqrDistanceToSegment(const Vector3d& P0, const Vector3d& P1) const;
};