#pragma once

#include <stdint.h>
#include <vector>

#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"
#include "ShapeType.h"

namespace Riemann
{
	class Triangle3d
	{
	public:
		Vector3 v0, v1, v2;

	public:
		Triangle3d()
		{
		}

		Triangle3d(const Vector3& InA, const Vector3& InB, const Vector3& InC)
		{
			Init(InA, InB, InC);
		}

		void	Init(const Vector3& InA, const Vector3& InB, const Vector3& InC)
		{
			v0 = InA;
			v1 = InB;
			v2 = InC;
		}

		Vector3& operator[](int i)
		{
			return (&v0)[i];
		}

		const Vector3& operator[](int i) const
		{
			return (&v0)[i];
		}

		Vector3		GetNormal() const
		{
			return GetNormal(v0, v1, v2);
		}

		Vector3		GetSideLength() const
		{
			return Vector3((v0 - v1).Length(), (v1 - v2).Length(), (v2 - v0).Length());
		}

		bool		IsValid() const
		{
			Vector3 sides = GetSideLength();
			bool v = sides.x + sides.y > sides.z &&
				sides.y + sides.z > sides.x &&
				sides.z + sides.x > sides.y;
			return v;
		}

		static Vector3 GetNormal(const Vector3& A, const Vector3& B, const Vector3& C)
		{
			Vector3 BA = (B - A).Unit();
			Vector3 CA = (C - A).Unit();
			Vector3 cross = BA.Cross(CA);
			return cross.Unit();
		}

		static bool		RayIntersectTriangle(const Vector3& Origin,
			const Vector3& Direction,
			const Vector3& A,
			const Vector3& B,
			const Vector3& C,
			float* t);

		static bool		IsColinear(const Vector3& a, const Vector3& b, const Vector3& c);

		bool			IntersectPoint(const Vector3& Point) const;
		bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
		bool			IntersectSegment(const Vector3& P0, const Vector3& P1) const;
		bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
		static bool		IntersectAABB(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& Bmin, const Vector3& Bmax);
		bool			IntersectSphere(const Vector3& Center, float Radius) const;
		bool			IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const;
		bool			IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const;

		static Vector3	ClosestPointOnTriangleToPoint(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C);
		static Vector3	ClosestPointOnTriangleToPointEx(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C, unsigned char& mask);
		static float	SqrDistancePointToTriangle(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C);
		float			SqrDistanceToPoint(const Vector3& Point) const;
		Vector3			ClosestPointToPoint(const Vector3& Point) const;

		Vector3			BaryCentric2D(const Vector3& Point) const;
		Vector3			BaryCentric3D(const Vector3& Point) const;
		static float	TriangleArea3D(const Vector3& A, const Vector3& B, const Vector3& C);

		float			CalcArea() const
		{
			return Triangle3d::TriangleArea3D(v0, v1, v2);
		}

		Box3			CalculateBoundingVolume() const
		{
			Box3 box(v0, v0);
			box.Encapsulate(v1, v2);
			return box;
		}
	};

	struct Triangle3dTriangle3dIntersectionResult
	{
		int			Quantity{ 0 };
		Vector3		Points[6];
		IntersectionType Type{ IntersectionType::Empty };
	};

	bool CalculateIntersectionTriangle3dTriangle3d(const Triangle3d& triangle0, const Triangle3d& triangle1, Triangle3dTriangle3dIntersectionResult &Result);

	Vector3 ClosestPtTetrahedronToPoint(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d);
}