
#pragma once

#include "GeometricObject.h"

class Plane : public GeometricObject
{
public:
	Vector3d Normal;    //  P * Normal + D = 0
	float D;

public:
	Plane() {}

	Plane(const Vector3d& InNormal, float InD)
	{
		Normal = InNormal;
		D = InD;
	}

public:
	virtual bool			IntersectPoint(const Vector3d& Point) const override
	{
		const float det = Point.Dot(Normal) + D;

		if (fabsf(det) < kEpsilonGeometric)
		{
			return true;
		}
		return false;
	}

	virtual bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const override
	{
		const float det = Dir.Dot(Normal);
		if (det > -kEpsilonGeometric && det < kEpsilonGeometric) {
			return false;
		}
		*t = -(Origin.Dot(Normal) + D) / det;
		return *t >= 0.0;
	}

	virtual bool			IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const override
	{
		Vector3d v[8];
		BoundingBox3d::GetVertices(Bmin, Bmax, v);
		int p0 = 0, p1 = 0, p2 = 0;
		for (int i = 0; i < 8; ++i)
		{
			const float det = v[i].Dot(Normal) + D;
			if (det > kEpsilonGeometric)
				p2++;
			else if (det < -kEpsilonGeometric)
				p1++;
			else
				p0++;
		}

		if (p1 > 0 && p2 > 0)
		{
			return true;
		}

		if (p0 == 0)
		{
			return false;
		}

		return true;
	}

	virtual BoundingBox3d	GetBoundingBox() const override
	{
		BoundingBox3d Box(Vector3d::InfMin(), Vector3d::InfMax());
		if (ParallelToXY())
		{
			Box.Min.z = Box.Max.z = -D / Normal.z;
		}

		if (ParallelToYZ())
		{
			Box.Min.x = Box.Max.x = -D / Normal.x;
		}

		if (ParallelToXZ())
		{
			Box.Min.y = Box.Max.y = -D / Normal.y;
		}

		return Box;
	}

	bool PerpendicularTo(const Vector3d& Axis) const
	{
		float det = Normal.Cross(Axis).SquareLength();
		if (det < kEpsilonGeometric)
		{
			return true;
		}
		return false;
	}

	bool ParallelToXY() const
	{
		return PerpendicularTo(Vector3d::UnitZ());
	}

	bool ParallelToXZ() const
	{
		return PerpendicularTo(Vector3d::UnitY());
	}

	bool ParallelToYZ() const
	{
		return PerpendicularTo(Vector3d::UnitX());
	}
};