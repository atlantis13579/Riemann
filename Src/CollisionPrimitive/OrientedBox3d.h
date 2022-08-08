
#pragma once

#include "ShapeType.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"

class OrientedBox3d
{
public:
	Vector3	Center;
	Vector3	Extent;
	Matrix3	Rot;

	OrientedBox3d()
	{
	}

	OrientedBox3d(const Vector3& _Center, const Vector3& _Extent, const Matrix3& _Rot)
	{
		Center = _Center;
		Extent = _Extent;
		Rot = _Rot;
	}
	
	static OrientedBox3d CalcBoundingOBB_PCA(const Vector3 *points, int n);

	float SqrDistanceToPoint(const Vector3& Point) const;
	float SqrDistanceToLine(const Vector3& P0, const Vector3& Dir, float* t) const;
	float SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const;
};
