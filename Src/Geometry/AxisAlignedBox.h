
#pragma once

#include "GeometricObject.h"

class AxisAlignedBox3 : public GeometricObject
{
public:
	Vector3d Min;
	Vector3d Max;

public:
	virtual bool			IntersectPoint(const Vector3d& Point) const override
	{
		if (Point.x >= Min.x && Point.x <= Max.x &&
			Point.y >= Min.y && Point.y <= Max.y &&
			Point.z >= Min.z && Point.z <= Max.z)
		{
			return true;
		}
		return false;
	}

	virtual bool			IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const override
	{
		if (Min.x > Bmax.x || Bmin.x > Max.x)
		{
			return false;
		}

		if (Min.y > Bmax.y || Bmin.y > Max.y)
		{
			return false;
		}

		if (Min.z > Bmax.z || Bmin.z > Max.z)
		{
			return false;
		}

		return true;
	}

	static bool RayIntersectAABB_1D(float start, float dir, float min, float max, float* enter, float* exit)
	{
		if (fabs(dir) < kEpsilonGeometric)
		{
			return start >= min && start <= max;
		}

		float   invDir = 1.0f / dir;
		float   t0 = (min - start) * invDir;
		float   t1 = (max - start) * invDir;

		if (t0 > t1)
		{
			std::swap(t0, t1);
		}

		if (t0 > *exit || t1 < *enter)
		{
			return false;
		}

		if (t0 > *enter)
		{
			*enter = t0;
		}

		if (t1 < *exit)
		{
			*exit = t1;
		}

		return true;
	}

	virtual bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const override
	{
		float enter = 0.0f, exit = 1.0f;

		for (int i = 0; i < 3; ++i)
		{
			if (!RayIntersectAABB_1D(Origin(i), Dir(i), Min(i), Max(i), &enter, &exit))
			{
				return false;
			}
		}

		const float h = enter > 0 ? enter : exit;
		if (h >= 0)
		{
			*t = h;
			return true;
		}
		return false;
	}
	
	virtual BoundingBox3d	GetBoundingBox() const override
	{
		return BoundingBox3d(Min, Max);
	}
};