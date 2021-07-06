
#pragma once

#include "../Maths/Vector3d.h"

class Ray
{
public:
	Ray() {}

	Ray(const Vector3d& InOrigin, const Vector3d& InDir, bool bNormalize = false)
	{
		Origin = InOrigin;
		Dir = InDir;
		if (bNormalize)
		{
			Dir.Normalize();
		}
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

	bool  IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax, float* t) const
	{
		float x = 1.0f / Dir.x;
		float y = 1.0f / Dir.y;
		float z = 1.0f / Dir.z;

		float t1 = (Bmin.x - Origin.x) * x;
		float t2 = (Bmax.x - Origin.x) * x;
		float t3 = (Bmin.y - Origin.y) * y;
		float t4 = (Bmax.y - Origin.y) * y;
		float t5 = (Bmin.z - Origin.z) * z;
		float t6 = (Bmax.z - Origin.z) * z;

		float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
		float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));


		if (tmax < 0)
		{
			*t = tmax;
			return false;
		}

		if (tmin > tmax)
		{
			*t = tmax;
			return false;
		}

		*t = tmin;
		return tmin >= 0.0f;
	}

	Vector3d Origin;
	Vector3d Dir;
};