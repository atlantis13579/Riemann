
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
		const Vector3d Vmin = Bmin - Origin;
		const Vector3d Vmax = Bmax - Origin;

		float Tmin = 0;
		float Tmax = FLT_MAX;
		Vector3d Normal(0.0f);

		for (int i = 0; i < 3; ++i)
		{
			float Time1, Time2;
			if (fabsf(Dir[i]) < 0.00001f)
			{
				if (Vmin[i] > 0 || Vmax[i] < 0)
				{
					return false;
				}
				else
				{
					Time1 = 0;
					Time2 = FLT_MAX;
				}
			}
			else
			{
				const float InvDir = 1.0f / Dir[i];
				Time1 = Vmin[i] * InvDir;
				Time2 = Vmax[i] * InvDir;
			}

			Vector3d CurNormal = Vector3d(0.0f);
			CurNormal[i] = 1.0f;

			if (Time1 > Time2)
			{
				std::swap(Time1, Time2);
			}
			else
			{
				CurNormal[i] = -1.0f;
			}

			if (Time1 > Tmin)
			{
				Normal = CurNormal;
			}
			Tmin = std::max(Tmin, Time1);
			Tmax = std::min(Tmax, Time2);

			if (Tmin > Tmax)
			{
				return false;
			}
		}

		if (Tmax < 0)
		{
			return false;
		}

		*t = Tmin;
		return true;
	}

	Vector3d Origin;
	Vector3d Dir;
};