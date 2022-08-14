#pragma once

#include <stdint.h>
#include <vector>

#include "ShapeType.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Box3d.h"

class AxisAlignedBox3d
{
public:
	Vector3 Min;
	Vector3 Max;

public:
	AxisAlignedBox3d()
	{
	}

	AxisAlignedBox3d(const Vector3& Bmin, const Vector3& Bmax)
	{
		Min = Bmin;
		Max = Bmax;
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::BOX;
	}

public:
	bool			IntersectPoint(const Vector3& Point) const;
	bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
	bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
	bool			IntersectSegment(const Vector3& P0, const Vector3& P1) const;
	bool			IntersectSphere(const Vector3& Center, float Radius) const;
	bool			IntersectCapsule(const Vector3& P0, const Vector3& P1, float Radius) const;
	bool			IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const;

	Vector3			ClosestPointToPoint(const Vector3& Point) const;
	float			SqrDistanceToPoint(const Vector3& Point) const;
	float			SqrDistanceToLine(const Vector3& P0, const Vector3& Direction, float* t) const;
	float			SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const;

	Vector3			GetCenter() const
	{
		return (Max + Min) * 0.5f;
	}

	Vector3			GetExtent() const
	{
		return (Max - Min) * 0.5f;
	}

	Box3d			GetBoundingVolume() const
	{
		return Box3d(Min, Max);
	}

	float			GetVolume() const
	{
		return GetVolume(Min, Max);
	}

	static float	GetVolume(const Vector3& Bmin, const Vector3& Bmax)
	{
		return ((Bmax.x - Bmin.x) * (Bmax.y - Bmin.y) * (Bmax.z - Bmin.z));
	}

	Vector3		GetCenterOfMass() const
	{
		return (Max + Min) * 0.5f;
	}

	Matrix3		GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Min, Max, Mass);
	}

	static Matrix3 GetInertiaTensor(const Vector3& Bmin, const Vector3& Bmax, float Mass)
	{
		Vector3 Dim = Bmax - Bmin;

		// https://www.wolframalpha.com/input/?i=cuboid
		const float M = Mass / 12;
		const float WW = Dim.x * Dim.x;
		const float HH = Dim.y * Dim.y;
		const float DD = Dim.z * Dim.z;
		return Matrix3(M * (HH + DD), M * (WW + DD), M * (WW + HH));
	}

	Vector3			GetSupport(const Vector3& Direction) const;
	static Vector3	GetSupport(const Vector3& Bmin, const Vector3& Bmax, const Vector3& Direction);
	int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const;
	static int		GetSupportFace(const Vector3& Bmin, const Vector3& Bmax, const Vector3& Direction, Vector3* FacePoints);

	void			GetMesh2(std::vector<Vector3> &Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);
	void			GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);
	void			GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices);
};
