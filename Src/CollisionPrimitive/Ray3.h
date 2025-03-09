#pragma once

#include "../Maths/Vector3.h"

class Ray3
{
public:
	Vector3 Origin;
	Vector3 Dir;

	Ray3() {}

	Ray3(const Vector3& InOrigin, const Vector3& InDir, bool bNormalize = false)
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

	static bool		RayIntersectAABB(const Vector3& Origin, const Vector3& Direction, const Vector3& Bmin, const Vector3& Bmax, float* t)
	{
		float enter = 0.0f, exit = 1.0f;

		for (int i = 0; i < 3; ++i)
		{
			if (!RayIntersectAABB_1D(Origin[i], Direction[i], Bmin[i], Bmax[i], &enter, &exit))
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

	static bool		RayIntersectAABB2(const Vector3& Origin, const Vector3& Direction, const Vector3& Bmin, const Vector3& Bmax, float thickness, float maxDist, float* t0, float* t1)
	{
		const float kEpsilon = 1e-9f;

		Vector3 invD;
		invD.x = 1.0f / (std::max(fabsf(Direction.x), kEpsilon) * (Direction.x >= 0 ? 1.0f : -1.0f));
		invD.y = 1.0f / (std::max(fabsf(Direction.y), kEpsilon) * (Direction.y >= 0 ? 1.0f : -1.0f));
		invD.z = 1.0f / (std::max(fabsf(Direction.z), kEpsilon) * (Direction.z >= 0 ? 1.0f : -1.0f));

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

    bool    IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C, float* t)
    {
        const float kEpsilon = 0.0000001f;

        const Vector3 BA = B - A;
        const Vector3 CA = C - A;
        const Vector3 P = Dir.Cross(CA);
        const float det = BA.Dot(P);

        if (det > -kEpsilon && det < kEpsilon)
        {
            return false;
        }
        const float inv_det = 1.f / det;

        const Vector3 T = Origin - A;
        const float u = T.Dot(P) * inv_det;
        if (u < 0.f || u > 1.f)
        {
            return false;
        }

        const Vector3 Q = T.Cross(BA);
        const float v = Dir.Dot(Q) * inv_det;
        if (v < 0.f || u + v  > 1.f)
        {
            return false;
        }

        const float w = CA.Dot(Q) * inv_det;
        if (w > kEpsilon)
        {
            *t = w;
            return true;
        }

        return false;
    }
};
