
#pragma once

#include "../Maths/Matrix3d.h"
#include "../Maths/BoundingBox3d.h"

class AxisAlignedBox3
{
public:
	Vector3d Min;
	Vector3d Max;

public:
	AxisAlignedBox3(const Vector3d& Bmin, const Vector3d& Bmax)
	{
		Min = Bmin;
		Max = Bmax;
	}

public:
	bool			IntersectPoint(const Vector3d& Point) const
	{
		if (Point.x >= Min.x && Point.x <= Max.x &&
			Point.y >= Min.y && Point.y <= Max.y &&
			Point.z >= Min.z && Point.z <= Max.z)
		{
			return true;
		}
		return false;
	}

	bool			IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const
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

	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
	{
		return RayIntersectAABB(Origin, Dir, Min, Max, t);
	}

	static bool		RayIntersectAABB(const Vector3d& Origin, const Vector3d& Dir, const Vector3d& Bmin, const Vector3d& Bmax, float* t)
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

	BoundingBox3d	GetBoundingBox() const
	{
		return BoundingBox3d(Min, Max);
	}

	float			GetVolume() const
	{
		return GetVolume(Min, Max);
	}

	static float	GetVolume(const Vector3d& Bmin, const Vector3d& Bmax)
	{
		return ((Bmax.x - Bmin.x) * (Bmax.y - Bmin.y) * (Bmax.z - Bmin.z));
	}

	Vector3d GetCenterOfMass() const
	{
		return (Max + Min) * 0.5f;
	}

	Matrix3d GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Mass, Min, Max);
	}

	static Matrix3d GetInertiaTensor(float Mass, const Vector3d& Bmin, const Vector3d& Bmax)
	{
		Vector3d Dim = Bmax - Bmin;

		// https://www.wolframalpha.com/input/?i=cuboid
		const float M = Mass / 12;
		const float WW = Dim.x * Dim.x;
		const float HH = Dim.y * Dim.y;
		const float DD = Dim.z * Dim.z;
		return Matrix3d(M * (HH + DD), M * (WW + DD), M * (WW + HH));
	}
};