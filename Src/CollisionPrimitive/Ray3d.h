
#pragma once

#include "../Maths/Vector3.h"

class Ray3d
{
public:
	Vector3 Origin;
	Vector3 Dir;

	Ray3d() {}

	Ray3d(const Vector3& InOrigin, const Vector3& InDir, bool bNormalize = false)
	{
		Origin = InOrigin;
		Dir = InDir;
		if (bNormalize)
		{
			Dir.Normalize();
		}
	}

	void			Init(const Vector3& Src, const Vector3& Dst)
	{
		Origin = Src;
		Dir = Dst - Src;
		Dir.Normalize();
	}

	Vector3		PointAt(float t) const
	{
		return Origin + Dir * t;
	}

	bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax, float* t) const
	{
		return RayIntersectAABB(Origin, Dir, Bmin, Bmax, t);
	}

	static bool		RayIntersectAABB_1D(float start, float dir, float min, float max, float* enter, float* exit)
	{
		if (fabsf(dir) < 0.000001f)
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

	static bool		RayIntersectAABB(const Vector3& Origin, const Vector3& Dir, const Vector3& Bmin, const Vector3& Bmax, float* t)
	{
		float enter = 0.0f, exit = 1.0f;

		for (int i = 0; i < 3; ++i)
		{
			if (!RayIntersectAABB_1D(Origin[i], Dir[i], Bmin[i], Bmax[i], &enter, &exit))
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

	static bool		RayIntersectAABB2(const Vector3& Origin, const Vector3& Dir, const Vector3& Bmin, const Vector3& Bmax, float thickness, float maxDist, float* t0, float* t1)
	{
		const float kEpsilon = 1e-9f;

		Vector3 invD;
		invD.x = 1.0f / (std::max(fabsf(Dir.x), kEpsilon) * (Dir.x >= 0 ? 1.0f : -1.0f));
		invD.y = 1.0f / (std::max(fabsf(Dir.y), kEpsilon) * (Dir.y >= 0 ? 1.0f : -1.0f));
		invD.z = 1.0f / (std::max(fabsf(Dir.z), kEpsilon) * (Dir.z >= 0 ? 1.0f : -1.0f));

		Vector3 b0 = (Bmin - thickness - Origin) * invD;
		Vector3 b1 = (Bmax + thickness - Origin) * invD;

		Vector3 tMin = b0.Min(b1);
		Vector3 tMax = b0.Max(b1);

		float maxIn = std::max(tMin.x, std::max(tMin.y, tMin.z));
		float minOut = std::min(tMax.x, std::min(tMax.y, tMax.z));

		*t0 = std::max(maxIn, 0.0f);
		*t1 = std::min(minOut, maxDist);

		return *t0 < *t1;
	}

};