
#pragma once

#include "../Maths/Vector3d.h"

class Ray
{
public:
	Ray() {}

	Ray(const Vector3d& InOrigin, const Vector3d& InDir)
	{
		Origin = InOrigin;
		Dir = InDir;
	}

	void Construct(const Vector3d& Src, const Vector3d& Dst)
	{
		Origin = Src;
		Dir = Dst - Src;
		Dir.Normalize();
	}

	Vector3d PointAt(float t) const
	{
		return Origin + Dir * t;
	}

	Vector3d Origin;
	Vector3d Dir;
};