#pragma once

#include <vector>

#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"
#include "PrimitiveType.h"

namespace Riemann
{
	class Triangle3
	{
	public:
		Vector3 v0, v1, v2;

	public:
		Triangle3()
		{
		}

		Triangle3(const Vector3& InA, const Vector3& InB, const Vector3& InC)
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

		const Vector3* GetVertex() const
		{
			return &v0;
		}

		Vector3* GetVertex()
		{
			return &v0;
		}

		Vector3		GetNormal(bool bHighPrecisionNormal = false) const
		{
			return CalculateNormal(v0, v1, v2, bHighPrecisionNormal);
		}

		Vector3		GetCenter() const
		{
			return (v0 + v1 + v2) / 3.0f;
		}

		float		GetArea() const
		{
			return Triangle3::CalculateArea3D(v0, v1, v2);
		}

		Box3		GetBounds() const
		{
			Box3 box(v0, v1);
			box.Encapsulate(v2);
			return box;
		}

		Vector3		GetSideLength() const
		{
			return Vector3((v0 - v1).Length(), (v1 - v2).Length(), (v2 - v0).Length());
		}

		bool		IsDegenerate() const
		{
			Vector3 sides = GetSideLength();
			bool v = sides.x + sides.y > sides.z &&
				sides.y + sides.z > sides.x &&
				sides.z + sides.x > sides.y;
			return !v;
		}

		static Vector3 CalculateNormal(const Vector3& A, const Vector3& B, const Vector3& C, bool bHighPrecisionNormal)
		{
			if (bHighPrecisionNormal)
			{
				Vector3 CA = (C - A).Unit();
				Vector3 BA = (B - A).Unit();
				return CA.Cross(BA).Unit();
			}
			else
			{
				Vector3 CA = C - A;
				Vector3 BA = B - A;
				return CA.Cross(BA).Unit();
			}
		}

		static float CalculateArea3D(const Vector3& A, const Vector3& B, const Vector3& C)
		{
			float cx = (B.y - A.y) * (C.z - A.z) - (C.y - A.y) * (B.z - A.z);
			float cy = (B.z - A.z) * (C.x - A.x) - (C.z - A.z) * (B.x - A.x);
			float cz = (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y);
			return 0.5f * sqrtf(cx * cx + cy * cy + cz * cz);
		}

		static bool IsColinear(const Vector3& a, const Vector3& b, const Vector3& c)
		{
			Vector3 n = (b - a).Cross(c - a);
			return n.SquareLength() < 1e-6f;
		}
        
        static float ComputeAlignmentValue(const Vector3& triNormal, const Vector3& unitDir)
        {
            return -fabsf(triNormal.Dot(unitDir));
        }
        
        // determines if a newly touched triangle is "better" than best one so far.
        // In this context "better" means either clearly smaller impact distance, or a similar impact
        //  distance but a normal more aligned with the sweep direction.
        static bool IsBetterTriangle(float Distance, float AlignmentValue, float bestDistance, float bestAlignmentValue)
        {
            const float distEpsilon = 1e-3f * std::max(1.0f, std::max(Distance, bestDistance));

            if (Distance < bestDistance - distEpsilon)
                return true;

            if (Distance < bestDistance+distEpsilon && AlignmentValue < bestAlignmentValue)
                return true;

            if (AlignmentValue == bestAlignmentValue && Distance < bestDistance)
                return true;

            if (Distance == 0.0f)
                return true;

            return false;
        }

		static bool RayIntersectTriangle(const Vector3& Origin, const Vector3& Direction, const Vector3& A, const Vector3& B, const Vector3& C, float* t);
        static int RayIntersectTriangle2(const Vector3& Origin, const Vector3& Direction, const Vector3& vert0, const Vector3& edge1, const Vector3& edge2, float *t, float *u, float *v);
		bool IntersectPoint(const Vector3& Point) const;
		bool IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
		bool IntersectSegment(const Vector3& P0, const Vector3& P1) const;
		static bool IntersectAABB(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& Bmin, const Vector3& Bmax);
		bool IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
		bool IntersectSphere(const Vector3& Center, float Radius) const;
		bool IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const;
		bool IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const;

		struct Segment3IntersectionResult
		{
			int				Quantity{ 0 };
			Vector3			Points[2];
			IntersectionType Type{ IntersectionType::Empty };
		};
		bool IntersectSegmentEx(const Vector3& P0, const Vector3& P1, Segment3IntersectionResult &Result) const;

		struct Triangle3IntersectionResult
		{
			int				Quantity{ 0 };
			Vector3			Points[6];
			IntersectionType Type{ IntersectionType::Empty };
		};
		bool IntersectTriangle(const Triangle3& triangle1, Triangle3IntersectionResult& Result) const;

		void ProjectOntoAxis(const Vector3& axis, float& fmin, float& fmax) const;

		static Vector3 ClosestPointOnTriangleToPoint(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C);

		static float SqrDistancePointToTriangle(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C);

		float SqrDistanceToPoint(const Vector3& Point) const
		{
			return Triangle3::SqrDistancePointToTriangle(Point, v0, v1, v2);
		}

		float DistanceToPoint(const Vector3& Point) const
		{
			float sqrDist = SqrDistanceToPoint(Point);
			return sqrtf(sqrDist);
		}

		Vector3 ClosestPointToPoint(const Vector3& Point) const
		{
			return Triangle3::ClosestPointOnTriangleToPoint(Point, v0, v1, v2);
		}

		inline Vector3 BarycentricCoodsComposion(const Vector3& BaryCoords) const
		{
			return BaryCoords.x * v0 + BaryCoords.y * v1 + BaryCoords.z * v2;
		}

		// Assume Point inside Triangle Plane !!!!!!
		// Point = x*v0 + y*v1 + z*v2 = v2 + x * (v0 - v2) + y * (v1 - v2)
		inline Vector3 BarycentricCoods(const Vector3& Point) const
		{
			return BarycentricCoods(Point, v0, v1, v2);
		}

		// Assume Point inside Triangle Plane !!!!!!
		// Point = x*v0 + y*v1 + z*v2 = v2 + x * (v0 - v2) + y * (v1 - v2)
		static Vector3 BarycentricCoods(const Vector3& Point, const Vector3& v0, const Vector3& v1, const Vector3& v2)
		{
			Vector3 v02 = v0 - v2;
			Vector3 v12 = v1 - v2;
			Vector3 vp2 = Point - v2;
			float d00 = v02.Dot(v02);
			float d01 = v02.Dot(v12);
			float d11 = v12.Dot(v12);
			float d20 = v02.Dot(vp2);
			float d21 = v12.Dot(vp2);
			float denom = d00 * d11 - d01 * d01;
			float u = (d11 * d20 - d01 * d21) / denom;
			float v = (d00 * d21 - d01 * d20) / denom;
			float w = 1.0f - u - v;
			return Vector3(u, v, w);
		}

		struct PointDistanceQueryResult
		{
			float	SqrDistance;
			Vector3	ClosestPoint;
			Vector3	BaryCoords;
		};

		PointDistanceQueryResult PointDistanceQuery(const Vector3 &Point) const;

		bool ContainsPoint(const Vector3& point) const;
	};
}
