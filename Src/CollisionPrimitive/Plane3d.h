#pragma once

#include <stdint.h>
#include <vector>

#include "ShapeType.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Box3d.h"

const float kEpsilonPlane = 0.000001f;
const float kHalfThickness = 1.0f;
const float kPlaneSmallThickness = 0.0001f;
const float kPlaneRadius = 10000.0f;

class Plane3d
{
public:
	Vector3 Normal;    //  P * Normal + D = 0
	float 	D;

public:
	Plane3d()
	{
	}

	Plane3d(const Vector3& InNormal, Vector3& InOrigin)
	{
		Normal = InNormal.Unit();
		D = -InOrigin.Dot(Normal);
	}

	Plane3d(const Vector3& InNormal, float InD)
	{
		Normal = InNormal.Unit();
		D = InD;
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::PLANE;
	}

public:
	Vector3			GetOrigin() const
	{
		return -Normal * D;
	}

	bool			IntersectPoint(const Vector3& Point) const
	{
		const float det = Point.Dot(Normal) + D;

		if (fabsf(det) < kEpsilonPlane)
		{
			return true;
		}
		return false;
	}

	bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		return RayIntersectPlane(Origin, Direction, Normal, D, t);
	}

	static bool		RayIntersectPlane(const Vector3& Origin, const Vector3& Direction, const Vector3 & Normal, float D, float* t)
	{
		const float det = Direction.Dot(Normal);
		if (det > -kEpsilonPlane && det < kEpsilonPlane) {
			return false;
		}
		*t = -(Origin.Dot(Normal) + D) / det;
		return *t >= 0.0;
	}

	bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
	{
		Vector3 v[8];
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
	
	static float	SignedDistanceToPlane(const Vector3& Point, const Vector3& Normal, const Vector3& Origin)
	{
		float signedDist = (Point - Origin).Dot(Normal);
		return signedDist;
	}
	
	static Vector3	ProjectToPlane(const Vector3& Point, const Vector3& Normal, const Vector3& Origin)
	{
		float signedDist = (Point - Origin).Dot(Normal);
		return Point - signedDist * Normal;
	}

	float			SignedDistanceTo(const Vector3& Point) const
	{
		Vector3 Origin = GetOrigin();
		return SignedDistanceToPlane(Point, Normal, Origin);
	}
	
	float			DistanceToPoint(const Vector3& Point) const
	{
		float Dist = SignedDistanceTo(Point);
		return fabsf(Dist);
	}

	Vector3			ClosestPointTo(const Vector3& Point) const
	{
		float SignedDist = SignedDistanceTo(Point);
		return Point - SignedDist * Normal;
	}

	float			DistanceToSegment(const Vector3& P0, const Vector3& P1) const
	{
		float SignedDist0 = SignedDistanceTo(P0);
		float SignedDist1 = SignedDistanceTo(P1);
		if (SignedDist0 * SignedDist1 <= 0.0f)
		{
			return 0.0f;
		}
		return std::min(fabsf(SignedDist0), fabsf(SignedDist1));
	}

	float			DistanceToTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
	{
		float sA = SignedDistanceTo(A);
		float sB = SignedDistanceTo(B);
		float sC = SignedDistanceTo(C);
		if ((sA > 0.0f && sB > 0.0f && sC > 0.0f) || (sA < 0.0f && sB < 0.0f && sC < 0.0f))
		{
			return std::min(fabsf(sA), std::min(fabsf(sB), fabsf(sC)));
		}
		return 0.0f;
	}

	bool			IntersectSphere(const Vector3& rCenter, float rRadius) const
	{
		float Dist = DistanceToPoint(rCenter);
		return Dist <= rRadius;
	}

	bool			IntersectCapsule(const Vector3& P0, const Vector3& P1, float Radius) const
	{
		float Dist = DistanceToSegment(P0, P1);
		return Dist <= Radius;
	}

	bool			IntersectPlane(const Vector3& Normal, float D) const
	{
		bool Parallel = Normal.ParallelTo(Normal);
		return !Parallel;
	}

	bool			IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
	{
		float Dist = DistanceToTriangle(A, B, C);
		return Dist > 0.0f;
	}

	static float	PlaneRadius()
	{
		return kPlaneRadius;
	}

	static float	VerySmallThickness()
	{
		return kPlaneSmallThickness;
	}

	Box3d			GetBoundingVolume() const
	{
		const float kMaxBV = PlaneRadius();
		Box3d Box(-kMaxBV, kMaxBV);

		if (ParallelToXY())
		{
			if (Normal.z > kEpsilonPlane)
			{
				Box.mMin.z = -D / Normal.z - kHalfThickness;
				Box.mMax.z = -D / Normal.z + kHalfThickness;
			}
		}

		if (ParallelToYZ())
		{
			if (Normal.x > kEpsilonPlane)
			{
				Box.mMin.x = -D / Normal.x - kHalfThickness;
				Box.mMax.x = -D / Normal.x + kHalfThickness;
			}
		}

		if (ParallelToXZ())
		{
			if (Normal.y > kEpsilonPlane)
			{
				Box.mMin.y = -D / Normal.y - kHalfThickness;
				Box.mMax.y = -D / Normal.y + kHalfThickness;
			}
		}

		return Box;
	}

	bool			PerpendicularTo(const Vector3& Axis) const
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
		return PerpendicularTo(Vector3::UnitZ());
	}

	bool			ParallelToXZ() const
	{
		return PerpendicularTo(Vector3::UnitY());
	}

	bool			ParallelToYZ() const
	{
		return PerpendicularTo(Vector3::UnitX());
	}

	Matrix3		GetInertiaTensor(float Mass) const
	{
		// TODO
		return Matrix3(1, 1, 1);
	}

	Vector3		GetSupport(const Vector3& Direction) const
	{
		Box3d box = GetBoundingVolume();
		return Vector3(
			Direction.x > 0 ? box.mMax.x : box.mMin.x,
			Direction.y > 0 ? box.mMax.y : box.mMin.y,
			Direction.z > 0 ? box.mMax.z : box.mMin.z
		);
	}

	int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
	{
		Box3d box = GetBoundingVolume();
		const Vector3& Bmin = box.mMin;
		const Vector3& Bmax = box.mMax;

		int axis = Direction.Abs().LargestAxis();
		if (Direction[axis] < 0.0f)
		{
			switch (axis)
			{
			case 0:
				FacePoints[0] = Vector3(Bmax.x, Bmin.y, Bmin.z);
				FacePoints[1] = Vector3(Bmax.x, Bmax.y, Bmin.z);
				FacePoints[2] = Vector3(Bmax.x, Bmax.y, Bmax.z);
				FacePoints[3] = Vector3(Bmax.x, Bmin.y, Bmax.z);
				break;

			case 1:
				FacePoints[0] = Vector3(Bmin.x, Bmax.y, Bmin.z);
				FacePoints[1] = Vector3(Bmin.x, Bmax.y, Bmax.z);
				FacePoints[2] = Vector3(Bmax.x, Bmax.y, Bmax.z);
				FacePoints[3] = Vector3(Bmax.x, Bmax.y, Bmin.z);
				break;

			case 2:
				FacePoints[0] = Vector3(Bmin.x, Bmin.y, Bmax.z);
				FacePoints[1] = Vector3(Bmax.x, Bmin.y, Bmax.z);
				FacePoints[2] = Vector3(Bmax.x, Bmax.y, Bmax.z);
				FacePoints[3] = Vector3(Bmin.x, Bmax.y, Bmax.z);
				break;
			}
		}
		else
		{
			switch (axis)
			{
			case 0:
				FacePoints[0] = Vector3(Bmin.x, Bmin.y, Bmin.z);
				FacePoints[1] = Vector3(Bmin.x, Bmin.y, Bmax.z);
				FacePoints[2] = Vector3(Bmin.x, Bmax.y, Bmax.z);
				FacePoints[3] = Vector3(Bmin.x, Bmax.y, Bmin.z);
				break;

			case 1:
				FacePoints[0] = Vector3(Bmin.x, Bmin.y, Bmin.z);
				FacePoints[1] = Vector3(Bmax.x, Bmin.y, Bmin.z);
				FacePoints[2] = Vector3(Bmax.x, Bmin.y, Bmax.z);
				FacePoints[3] = Vector3(Bmin.x, Bmin.y, Bmax.z);
				break;

			case 2:
				FacePoints[0] = Vector3(Bmin.x, Bmin.y, Bmin.z);
				FacePoints[1] = Vector3(Bmin.x, Bmax.y, Bmin.z);
				FacePoints[2] = Vector3(Bmax.x, Bmax.y, Bmin.z);
				FacePoints[3] = Vector3(Bmax.x, Bmin.y, Bmin.z);
				break;
			}
		}

		return 4;
	}

	void			GetVerties(Vector3 *v, float Radius)
	{
		Vector3 v0 = Vector3::UnitY();
		if (Normal.ParallelTo(v0) != 0)
		{
			v0 = Vector3::UnitX();
		}

		Vector3 v1 = Normal.Cross(v0);
		Vector3 v2 = Normal.Cross(v1);

		Vector3 Origin = GetOrigin();
		v[0] = Origin + v1 * Radius;
		v[1] = Origin + v2 * Radius;
		v[2] = Origin + v1 * -Radius;
		v[3] = Origin + v2 * -Radius;
		return;
	}

	void			GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
	{
		Vertices.resize(4);
		GetVerties(&Vertices[0], 100.0f);
		Normals = { Normal , Normal , Normal , Normal };
		Indices = { 2,1,0, 2,3,0 };
	}

	void			GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
	{
		Vertices.resize(4);
		GetVerties(&Vertices[0], 100.0f);
		Indices = { 0,1, 1,2, 2,3, 3,0 };
	}
};

static_assert(sizeof(Plane3d) == 16, "sizeof(Plane3d) not right");
