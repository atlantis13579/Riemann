#pragma once

#include <stdint.h>
#include <vector>

#include "ShapeType.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Box3d.h"

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
	
	Plane3d(const Vector3& A, const Vector3& B, const Vector3& C)
	{
		Normal = (A - C).Cross(B - C);
		D = -C.Dot(Normal);
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

	bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
	bool			IntersectSegment(const Vector3& P0, const Vector3& P1) const;
	static bool		RayIntersectPlane(const Vector3& Origin, const Vector3& Direction, const Vector3 & Normal, float D, float* t);

	bool			IntersectPoint(const Vector3& Point) const;
	bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
	bool			IntersectSphere(const Vector3& rCenter, float rRadius) const;
	bool			IntersectCapsule(const Vector3& P0, const Vector3& P1, float Radius) const;
	bool			IntersectPlane(const Vector3& _Normal, float _D) const;
	bool			IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const;

	bool			PenetrateSphere(const Vector3 &Center, float Radius, Vector3 *normal, float *depth) const;
	
	static bool 	GetIntersection(const Plane3d& p1, const Plane3d& p2, Vector3 &Origin, Vector3 &Dir);
	static bool 	GetIntersection(const Plane3d& p1, const Plane3d& p2, const Plane3d& p3, Vector3 &p);
	static float	SignedDistanceToPlane(const Vector3& Point, const Vector3& Normal, const Vector3& Origin);
	static Vector3	ProjectToPlane(const Vector3& Point, const Vector3& Normal, const Vector3& Origin);
	float			SignedDistanceTo(const Vector3& Point) const;
	float			DistanceToPoint(const Vector3& Point) const;
	Vector3			ClosestPointTo(const Vector3& Point) const;
	float			DistanceToSegment(const Vector3& P0, const Vector3& P1) const;
	float			DistanceToTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const;

	float			GetVolume() const { return 0.0f; }
	Box3d			GetBoundingVolume() const;

	bool			PerpendicularTo(const Vector3& Axis) const;
	bool			ParallelToXY() const;
	bool			ParallelToXZ() const;
	bool			ParallelToYZ() const;

	Matrix3			GetInertiaTensor(float Mass) const
	{
		// TODO
		return Matrix3(1, 1, 1);
	}

	Vector3			GetSupport(const Vector3& Direction) const;

	int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const;

	void			GetVerties(Vector3 *v, float Radius);
	void			GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);
	void			GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices);
};

static_assert(sizeof(Plane3d) == 16, "sizeof(Plane3d) not right");
