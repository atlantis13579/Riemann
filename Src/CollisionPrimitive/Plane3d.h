
#pragma once

#include "../Maths/Vector3d.h"
#include "../Maths/Matrix3d.h"
#include "../Maths/Box3d.h"
#include "ShapeType.h"

const float kEpsilonPlane = 0.000001f;

class Plane3d
{
public:
	Vector3d Normal;    //  P * Normal + D = 0
	float D;

public:
	Plane3d()
	{
	}

	Plane3d(const Vector3d& InNormal, float InD)
	{
		Normal = InNormal.Unit();
		D = InD;
	}

	static constexpr ShapeType	StaticType()
	{
		return ShapeType::PLANE;
	}

public:
	Vector3d		GetOrigin() const
	{
		return -Normal * D;
	}

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
		Box3d::GetVertices(Bmin, Bmax, v);
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

	float			DistanceTo(const Vector3d& Point) const
	{
		float Dist = SignedDistanceTo(Point);
		return fabsf(Dist);
	}

	float			SignedDistanceTo(const Vector3d& Point) const
	{
		Vector3d Origin = GetOrigin();
		float Dist = (Point - Origin).Dot(Normal);
		return Dist;
	}

	Vector3d		ClosestPointTo(const Vector3d& Point) const
	{
		float SignedDist = SignedDistanceTo(Point);
		return Point - SignedDist * Normal;
	}

	float			DistanceToSegment(const Vector3d& P0, const Vector3d& P1) const
	{
		float SignedDist0 = SignedDistanceTo(P0);
		float SignedDist1 = SignedDistanceTo(P1);
		if (SignedDist0 * SignedDist1 <= 0.0f)
		{
			return 0.0f;
		}
		return std::min(fabsf(SignedDist0), fabsf(SignedDist1));
	}

	bool			IntersectSphere(const Vector3d& rCenter, float rRadius) const
	{
		float Dist = DistanceTo(rCenter);
		return Dist <= rRadius;
	}

	bool			IntersectCapsule(const Vector3d& P0, const Vector3d& P1, float Radius) const
	{
		float Dist = DistanceToSegment(P0, P1);
		return Dist <= Radius;
	}

	static float	MaxBV()
	{
		return 1000000.0f;
	}

	static float	VerySmallTickness()
	{
		return 0.00001f;
	}

	Box3d			GetBoundingVolume() const
	{
		const float kMaxBV = MaxBV();
		const float kVerySmallTickness = VerySmallTickness();
		Box3d Box(-kMaxBV, kMaxBV);

		if (ParallelToXY())
		{
			if (Normal.z > kEpsilonPlane)
			{
				Box.Min.z = -D / Normal.z - kVerySmallTickness;
				Box.Max.z = -D / Normal.z + kVerySmallTickness;
			}
		}

		if (ParallelToYZ())
		{
			if (Normal.x > kEpsilonPlane)
			{
				Box.Min.x = -D / Normal.x - kVerySmallTickness;
				Box.Max.x = -D / Normal.x + kVerySmallTickness;
			}
		}

		if (ParallelToXZ())
		{
			if (Normal.y > kEpsilonPlane)
			{
				Box.Min.y = -D / Normal.y - kVerySmallTickness;
				Box.Max.y = -D / Normal.y + kVerySmallTickness;
			}
		}

		return Box;
	}

	bool			PerpendicularTo(const Vector3d& Axis) const
	{
		float det = Normal.Cross(Axis).SquareLength();
		if (det < kEpsilonPlane)
		{
			return true;
		}
		return false;
	}

	bool			ParallelToXY() const
	{
		return PerpendicularTo(Vector3d::UnitZ());
	}

	bool			ParallelToXZ() const
	{
		return PerpendicularTo(Vector3d::UnitY());
	}

	bool			ParallelToYZ() const
	{
		return PerpendicularTo(Vector3d::UnitX());
	}

	Matrix3d		GetInertiaTensor(float Mass) const
	{
		// TODO
		return Matrix3d(1, 1, 1);
	}

	Vector3d		GetSupport(const Vector3d& dir) const
	{
		Box3d box = GetBoundingVolume();
		return Vector3d(
			dir.x > 0 ? box.Max.x : box.Min.x,
			dir.y > 0 ? box.Max.y : box.Min.y,
			dir.z > 0 ? box.Max.z : box.Min.z
		);
	}

	void			GetVerties(Vector3d *v, float Radius)
	{
		Vector3d v0 = Vector3d::UnitY();
		if (Normal.ParallelTo(v0) != 0)
		{
			v0 = Vector3d::UnitX();
		}

		Vector3d v1 = Normal.Cross(v0);
		Vector3d v2 = Normal.Cross(v1);

		Vector3d Origin = GetOrigin();
		v[0] = Origin + v1 * Radius;
		v[1] = Origin + v2 * Radius;
		v[2] = Origin + v1 * -Radius;
		v[3] = Origin + v2 * -Radius;
		return;
	}

	void			GetMesh(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3d>& Normals)
	{
		Vertices.resize(4);
		GetVerties(&Vertices[0], 100.0f);
		Normals = { Normal , Normal , Normal , Normal };
		Indices = { 2,1,0, 2,3,0 };
	}

	void			GetWireframe(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices)
	{
		Vertices.resize(4);
		GetVerties(&Vertices[0], 100.0f);
		Indices = { 0,1, 1,2, 2,3, 3,0 };
	}
};