
#pragma once

const float kEpsilonPlane = 0.000001f;

class Plane
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
	bool			IntersectPoint(const Vector3d& Point) const
	{
		const float det = Point.Dot(Normal) + D;

		if (fabsf(det) < kEpsilonPlane)
		{
			return true;
		}
		return false;
	}

	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
	{
		return RayIntersectPlane(Origin, Dir, Normal, D, t);
	}

	static bool		RayIntersectPlane(const Vector3d& Origin, const Vector3d& Dir, const Vector3d & Normal, float D, float* t)
	{
		const float det = Dir.Dot(Normal);
		if (det > -kEpsilonPlane && det < kEpsilonPlane) {
			return false;
		}
		*t = -(Origin.Dot(Normal) + D) / det;
		return *t >= 0.0;
	}

	bool			IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const
	{
		Vector3d v[8];
		BoundingBox3d::GetVertices(Bmin, Bmax, v);
		int p0 = 0, p1 = 0, p2 = 0;
		for (int i = 0; i < 8; ++i)
		{
			const float det = v[i].Dot(Normal) + D;
			if (det > kEpsilonPlane)
				p2++;
			else if (det < -kEpsilonPlane)
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

	BoundingBox3d	GetBoundingBox() const
	{
		const float kMaxBV = 10000000.0f;
		BoundingBox3d Box(-kMaxBV, kMaxBV);
		if (ParallelToXY())
		{
			if (Normal.z > 0.0001f)
				Box.Min.z = Box.Max.z = -D / Normal.z;
		}

		if (ParallelToYZ())
		{
			if (Normal.x > 0.0001f)
				Box.Min.x = Box.Max.x = -D / Normal.x;
		}

		if (ParallelToXZ())
		{
			if (Normal.y > 0.0001f)
				Box.Min.y = Box.Max.y = -D / Normal.y;
		}

		return Box;
	}

	bool PerpendicularTo(const Vector3d& Axis) const
	{
		float det = Normal.Cross(Axis).SquareLength();
		if (det < kEpsilonPlane)
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

	Matrix3d GetInertiaTensor(float Mass) const
	{
		// TODO
		return Matrix3d(1, 1, 1);
	}

	Vector3d GetSupport(const Vector3d& dir) const
	{
		BoundingBox3d box = GetBoundingBox();
		return Vector3d(
			dir.x > 0 ? box.Max.x : -box.Min.x,
			dir.y > 0 ? box.Max.y : -box.Min.y,
			dir.z > 0 ? box.Max.z : -box.Min.z
		);
	}
};