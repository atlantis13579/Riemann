#pragma once

#include <stdint.h>
#include <vector>

#include "../Maths/Box3d.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"
#include "ShapeType.h"

class Triangle3d
{
public:
	Vector3 A, B, C;

public:
	Triangle3d()
	{
	}

	Triangle3d(const Vector3& InA, const Vector3& InB, const Vector3& InC)
	{
		Init(InA, InB, InC);
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::TRIANGLE;
	}

	void			Init(const Vector3& InA, const Vector3& InB, const Vector3& InC)
	{
		A = InA;
		B = InB;
		C = InC;
	}

	Vector3 operator[](int i)
	{
		return (&A)[i];
	}

	const Vector3& operator[](int i) const
	{
		return (&A)[i];
	}

	Vector3		GetUnitNormal() const
	{
		return GetUnitNormal(A, B, C);
	}

	static Vector3 GetUnitNormal(const Vector3& A, const Vector3& B, const Vector3& C)
	{
		Vector3 cross = (B - A).Cross(C - A);
		return cross.Unit();
	}
	
	Vector3		GetNormal() const
	{
		return GetNormal(A, B, C);
	}
	
	static Vector3 GetNormal(const Vector3& A, const Vector3& B, const Vector3& C)
	{
		Vector3 cross = (B - A).Cross(C - A);
		return cross;
	}

	static bool		RayIntersectTriangle(const Vector3& Origin,
		const Vector3& Direction,
		const Vector3& A,
		const Vector3& B,
		const Vector3& C,
		float* t);

	bool			IntersectPoint(const Vector3& Point) const;
	bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
	bool			IntersectSegment(const Vector3& P0, const Vector3& P1) const;
	bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
	static bool		IntersectAABB(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& Bmin, const Vector3& Bmax);
	bool			IntersectSphere(const Vector3& Center, float Radius) const;
	bool			IntersectCapsule(const Vector3& X0, const Vector3 &X1, float Radius) const;

	static Vector3	ClosestPointOnTriangleToPoint(const Vector3 &Point, const Vector3 &A, const Vector3 &B, const Vector3 &C);
	static Vector3	ClosestPointOnTriangleToPointEx(const Vector3 &Point, const Vector3 &A, const Vector3 &B, const Vector3 &C, unsigned char &mask);
	static float	SqrDistancePointToTriangle(const Vector3 &Point, const Vector3 &A, const Vector3 &B, const Vector3 &C);
	float			SqrDistanceToPoint(const Vector3 &Point) const;
	Vector3			ClosestPointToPoint(const Vector3 &Point) const;
	
	Vector3			BaryCentric2D(const Vector3& Point) const;
	Vector3			BaryCentric3D(const Vector3& Point) const;
	static float	TriangleArea3D(const Vector3& A, const Vector3& B, const Vector3& C);
	
	float			CalcArea() const
	{
		return Triangle3d::TriangleArea3D(A, B, C);
	}
	
	bool CalculateVolumeProperties(MassParameters* p, float Density) const
	{
		p->Volume = 0.0f;
		p->Mass = 0.0f;
		p->CenterOfMass = (A + B + C) / 3.0f;
		p->BoundingVolume = CalculateBoundingVolume();
		p->InertiaMat = Matrix3::Identity();
		return true;
	}

	Box3d			CalculateBoundingVolume() const
	{
		Box3d box(A, A);
		box.Encapsulate(B, C);
		return box;
	}

	Vector3			GetSupport(const Vector3& Direction) const;
	static Vector3	GetSupport(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& Direction);
	int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const;

	void			GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);
	void			GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices);
};

Vector3 ClosestPtTetrahedronToPoint(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d);
