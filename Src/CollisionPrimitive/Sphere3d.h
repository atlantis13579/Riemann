
#pragma once

#include <math.h>
#include "../Maths/Matrix3d.h"

class Sphere3d
{
public:
	Vector3d Center;
	float Radius;

public:
	Sphere3d() {}

	Sphere3d(const Vector3d& InCenter, float InRadius)
	{
		Center = InCenter;
		Radius = InRadius;
	}

public:
	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
	{
		return RayIntersectSphere(Origin, Dir, Center, Radius, t);
	}

	static bool		RayIntersectSphere(const Vector3d& Origin, const Vector3d& Dir, const Vector3d& Center, float Radius, float* t)
	{
		Vector3d oc = Origin - Center;
		float a = Dir.SquareLength();
		float b = 2.0f * oc.Dot(Dir);
		float c = oc.SquareLength() - Radius * Radius;
		float discriminant = b * b - 4 * a * c;
		if (discriminant < 0)
		{
			return false;
		}
		float h = (-b - sqrtf(discriminant)) / (2.0f * a);
		if (h >= 0)
		{
			*t = h;
			return true;
		}
		return false;
	}

	bool			IntersectPoint(const Vector3d& Point) const
	{
		float sqr_dist = (Point - Center).SquareLength();
		if (sqr_dist <= Radius * Radius)
		{
			return true;
		}
		return false;
	}

	// AABB intersects with a solid sphere or not by Jim Arvo, in "Graphics Gems":
	bool			IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const
	{
		float dmin = 0.0f;
		for (int i = 0; i < 3; i++)
		{
			if (Center[i] < Bmin[i])
			{
				dmin += (Center[i] - Bmin[i]) * (Center[i] - Bmin[i]);
			}
			else if (Center[i] > Bmax[i])
			{
				dmin += (Center[i] - Bmax[i]) * (Center[i] - Bmax[i]);
			}
		}

		if (dmin <= Radius * Radius)
		{
			return true;
		}
		return false;
	}

	Box3d	GetBoundingVolume() const
	{
		Box3d Box;
		Box.BuildFromCenterAndExtent(Center, Vector3d(Radius));
		return Box;
	}

	float GetVolume() const
	{
		const float FourThirdsPI = (4.0f / 3.0f) * (float)M_PI;
		return FourThirdsPI * Radius * Radius * Radius;
	}

	Vector3d GetCenterOfMass() const
	{
		return Center;
	}

	Matrix3d GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Radius, Mass);
	}

	static Matrix3d GetInertiaTensor(float Radius, float Mass)
	{
		const float Diagonal = 2.0f * Mass * Radius * Radius / 5.0f;
		return Matrix3d(Diagonal, Diagonal, Diagonal);
	}

	Vector3d GetSupport(const Vector3d& dir) const
	{
		return GetSupport(Center, Radius, dir);
	}

	static Vector3d GetSupport(const Vector3d& Center, float Radius, const Vector3d& dir)
	{
		float distSqr = dir.SquareLength();
		if (distSqr <= 1e-6)
		{
			return Center;
		}
		Vector3d Normalized = dir / sqrtf(distSqr);
		return Center + Normalized * Radius;
	}
};