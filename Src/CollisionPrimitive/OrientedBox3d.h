
#pragma once

#include "../Maths/Matrix3d.h"
#include "../Maths/Box3d.h"

class OrientedBox3d
{
public:
	Vector3d Min;
	Vector3d Max;

public:
	OrientedBox3d(const Vector3d& Bmin, const Vector3d& Bmax)
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

	static bool		RayIntersectAABB_1D(float start, float dir, float min, float max, float* enter, float* exit)
	{
		if (fabs(dir) < 0.000001f)
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

	static bool		RayIntersectAABB(const Vector3d& Origin, const Vector3d& Dir, const Vector3d& Bmin, const Vector3d& Bmax, float* t)
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

	Box3d	GetBoundingBox() const
	{
		return Box3d(Min, Max);
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
		return GetInertiaTensor(Min, Max, Mass);
	}

	static Matrix3d GetInertiaTensor(const Vector3d& Bmin, const Vector3d& Bmax, float Mass)
	{
		Vector3d Dim = Bmax - Bmin;

		// https://www.wolframalpha.com/input/?i=cuboid
		const float M = Mass / 12;
		const float WW = Dim.x * Dim.x;
		const float HH = Dim.y * Dim.y;
		const float DD = Dim.z * Dim.z;
		return Matrix3d(M * (HH + DD), M * (WW + DD), M * (WW + HH));
	}

	Vector3d GetSupport(const Vector3d& dir) const
	{
		return GetSupport(Min, Max, dir);
	}

	static Vector3d GetSupport(const Vector3d& Bmin, const Vector3d& Bmax, const Vector3d& dir)
	{
		return Vector3d(
			dir.x > 0 ? Bmax.x : Bmin.x,
			dir.y > 0 ? Bmax.y : Bmin.y,
			dir.z > 0 ? Bmax.z : Bmin.z
		);
	}
};