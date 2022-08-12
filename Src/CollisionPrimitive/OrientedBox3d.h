#pragma once

#include "ShapeType.h"
#include "../Maths/Box3d.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"

class OrientedBox3d
{
public:
	Vector3	Center;
	Vector3	Extent;
	Matrix3	Rotation;

	OrientedBox3d()
	{
	}

	OrientedBox3d(const Vector3& _Center, const Vector3& _Extent, const Matrix3& _Rot)
	{
		Center = _Center;
		Extent = _Extent;
		Rotation = _Rot;
	}
	
	static OrientedBox3d	ComputeBoundingOBB_PCA(const Vector3 *points, int n);
	static Box3d			ComputeBoundingVolume(const Vector3& Center, const Vector3& Extent, const Matrix3& Rot);
	Box3d					ComputeBoundingVolume() const;

	bool IntersectOBB(const OrientedBox3d& obb) const;
	bool IntersectOBB(const Vector3& _Center, const Vector3& _Extent, const Matrix3& _Rot) const;
	bool IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;

	float SqrDistanceToPoint(const Vector3& Point) const;
	float SqrDistanceToLine(const Vector3& P0, const Vector3& Direction, float* t) const;
	float SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const;
};
