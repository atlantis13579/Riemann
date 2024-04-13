#pragma once

#include <vector>

#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"
#include "Plane3.h"
#include "Capsule3.h"
#include "Triangle2.h"
#include "ShapeType.h"

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

		Vector3		GetNormal() const
		{
			return GetNormal(v0, v1, v2);
		}

		Vector3		GetCenter() const
		{
			return (v0 + v1 + v2) / 3.0f;
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
			const bool bHighPrecisionNormal = false;
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

		float			CalculateArea() const
		{
			return Triangle3::CalculateArea3D(v0, v1, v2);
		}

		Box3			CalculateBoundingVolume() const
		{
			Box3 box(v0, v0);
			box.Encapsulate(v1, v2);
			return box;
		}

		// Moller CTrumbore intersection algorithm
		static bool RayIntersectTriangle(const Vector3& Origin, const Vector3& Direction, const Vector3& A, const Vector3& B, const Vector3& C, float* t)
		{
			const float kEpsilonTri = 0.0000001f;
			//Find vectors for two edges sharing V1
			const Vector3 e1 = B - A;
			const Vector3 e2 = C - A;
			//Begin calculating determinant - also used to calculate u parameter
			const Vector3 P = Direction.Cross(e2);
			//if determinant is near zero, ray lies in plane of triangle
			const float det = e1.Dot(P);
			//NOT CULLING
			if (det > -kEpsilonTri && det < kEpsilonTri)
			{
				return false;
			}
			const float inv_det = 1.f / det;

			//calculate distance from V1 to ray origin
			const Vector3 T = Origin - A;

			//Calculate u parameter and test bound
			const float u = T.Dot(P) * inv_det;
			//The intersection lies outside of the triangle
			if (u < 0.f || u > 1.f)
			{
				return false;
			}

			//Prepare to test v parameter
			const Vector3 Q = T.Cross(e1);

			//Calculate V parameter and test bound
			const float v = Direction.Dot(Q) * inv_det;
			//The intersection lies outside of the triangle
			if (v < 0.f || u + v  > 1.f)
			{
				return false;
			}

			const float out = e2.Dot(Q) * inv_det;

			if (out > kEpsilonTri) { //ray intersection
				*t = out;
				return true;
			}

			// No hit, no win
			return false;
		}

		static bool IsColinear(const Vector3& a, const Vector3& b, const Vector3& c)
		{
			Vector3 n = (b - a).Cross(c - a);
			return n.SquareLength() < 1e-6f;
		}

		// https://math.stackexchange.com/questions/4322/check-whether-a-point-is-within-a-3d-triangle
		bool IntersectPoint(const Vector3& Point) const
		{
			const Vector3 Normal = GetNormal();
			const float det = Point.Dot(Normal) - Normal.Dot(v0);
			const float kPointInPlaneTor = 1e-6f;
			if (fabsf(det) > kPointInPlaneTor)
			{
				return false;
			}

			Vector3 bc = BarycentricCoods(Point);
			return (0.0f <= bc.x && bc.x <= 1.0f) 
				&& (0.0f <= bc.y && bc.y <= 1.0f)
				&& (0.0f <= bc.z && bc.z <= 1.0f);
		}

		bool IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
		{
			return RayIntersectTriangle(Origin, Direction, v0, v1, v2, t);
		}

		bool IntersectSegment(const Vector3& P0, const Vector3& P1) const
		{
			Plane3 plane(v0, v1, v2);
			int sign0 = plane.CalculateRelationsToPoint(P0);
			int sign1 = plane.CalculateRelationsToPoint(P1);
			if (sign0 * sign1 > 0)
			{
				return false;
			}

			float t;
			Vector3 Direction = (P1 - P0).Unit();
			return IntersectRay(P0, Direction, &t);
		}

		struct Segment3IntersectionResult
		{
			int				Quantity{ 0 };
			Vector3			Points[2];
			IntersectionType Type{ IntersectionType::Empty };
		};

		bool IntersectSegmentEx(const Vector3& P0, const Vector3& P1, Segment3IntersectionResult &Result) const
		{
			Plane3 plane(v0, v1, v2);
			int sign0 = plane.CalculateRelationsToPoint(P0);
			int sign1 = plane.CalculateRelationsToPoint(P1);
			if (sign0 * sign1 > 0)
			{
				Result.Quantity = 0;
				Result.Type = IntersectionType::Empty;
				return false;
			}

			Result.Quantity = IntersectSegment_Coplanar(plane, P0, P1, Result.Points);
			if (Result.Quantity == 2)
			{
				Result.Type = IntersectionType::Segment;
				return true;
			}
			else if (Result.Quantity == 1)
			{
				Result.Type = IntersectionType::Point;
				return true;
			}
			
			Result.Type = IntersectionType::Empty;
			return true;
		}

		// By Tomas Akenine-Moller
		// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
		// -----
		static bool IntersectAABB(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& Bmin, const Vector3& Bmax)
		{
			Vector3 Center = (Bmax + Bmin) * 0.5f;
			Vector3 Extent = Bmax - Center;

			float min, max, p0, p1, p2, rad, fex, fey, fez;

			Vector3 v0 = A - Center;
			Vector3 v1 = B - Center;
			Vector3 v2 = C - Center;

			Vector3 e0 = v1 - v0;
			Vector3 e1 = v2 - v1;
			Vector3 e2 = v0 - v2;

			fex = fabsf(e0.x);
			fey = fabsf(e0.y);
			fez = fabsf(e0.z);

			p0 = e0.z * v0.y - e0.y * v0.z;
			p2 = e0.z * v2.y - e0.y * v2.z;
			if (p0 < p2)
			{
				min = p0;
				max = p2;
			}
			else
			{
				min = p2;
				max = p0;
			}

			rad = fez * Extent.y + fey * Extent.z;
			if (min > rad || max < -rad)
				return false;

			p0 = -e0.z * v0.x + e0.x * v0.z;
			p2 = -e0.z * v2.x + e0.x * v2.z;
			if (p0 < p2)
			{
				min = p0;
				max = p2;
			}
			else
			{
				min = p2;
				max = p0;
			}

			rad = fez * Extent.x + fex * Extent.z;
			if (min > rad || max < -rad)
				return false;

			p1 = e0.y * v1.x - e0.x * v1.y;
			p2 = e0.y * v2.x - e0.x * v2.y;
			if (p2 < p1)
			{
				min = p2;
				max = p1;
			}
			else
			{
				min = p1;
				max = p2;
			}

			rad = fey * Extent.x + fex * Extent.y;
			if (min > rad || max < -rad)
				return false;

			fex = fabsf(e1.x);
			fey = fabsf(e1.y);
			fez = fabsf(e1.z);

			p0 = e1.z * v0.y - e1.y * v0.z;
			p2 = e1.z * v2.y - e1.y * v2.z;
			if (p0 < p2)
			{
				min = p0;
				max = p2;
			}
			else
			{
				min = p2;
				max = p0;
			}

			rad = fez * Extent.y + fey * Extent.z;
			if (min > rad || max < -rad)
				return false;

			p0 = -e1.z * v0.x + e1.x * v0.z;
			p2 = -e1.z * v2.x + e1.x * v2.z;
			if (p0 < p2)
			{
				min = p0;
				max = p2;
			}
			else
			{
				min = p2;
				max = p0;
			}

			rad = fez * Extent.x + fex * Extent.z;
			if (min > rad || max < -rad)
				return false;

			p0 = e1.y * v0.x - e1.x * v0.y;
			p1 = e1.y * v1.x - e1.x * v1.y;
			if (p0 < p1)
			{
				min = p0;
				max = p1;
			}
			else
			{
				min = p1;
				max = p0;
			}

			rad = fey * Extent.x + fex * Extent.y;
			if (min > rad || max < -rad)
				return false;

			fex = fabsf(e2.x);
			fey = fabsf(e2.y);
			fez = fabsf(e2.z);

			p0 = e2.z * v0.y - e2.y * v0.z;
			p1 = e2.z * v1.y - e2.y * v1.z;
			if (p0 < p1)
			{
				min = p0;
				max = p1;
			}
			else
			{
				min = p1;
				max = p0;
			}

			rad = fez * Extent.y + fey * Extent.z;
			if (min > rad || max < -rad)
				return false;

			p0 = -e2.z * v0.x + e2.x * v0.z;
			p1 = -e2.z * v1.x + e2.x * v1.z;
			if (p0 < p1)
			{
				min = p0;
				max = p1;
			}
			else
			{
				min = p1;
				max = p0;
			}

			rad = fez * Extent.x + fex * Extent.z;
			if (min > rad || max < -rad)
				return false;

			p1 = e2.y * v1.x - e2.x * v1.y;
			p2 = e2.y * v2.x - e2.x * v2.y;
			if (p2 < p1)
			{
				min = p2;
				max = p1;
			}
			else
			{
				min = p1;
				max = p2;
			}

			rad = fey * Extent.x + fex * Extent.y;
			if (min > rad || max < -rad)
				return false;

			min = std::min(v0.x, std::min(v1.x, v2.x));
			max = std::max(v0.x, std::max(v1.x, v2.x));
			if (min > Extent.x || max < -Extent.x)
				return false;

			min = std::min(v0.y, std::min(v1.y, v2.y));
			max = std::max(v0.y, std::max(v1.y, v2.y));
			if (min > Extent.y || max < -Extent.y)
				return false;

			min = std::min(v0.z, std::min(v1.z, v2.z));
			max = std::max(v0.z, std::max(v1.z, v2.z));
			if (min > Extent.z || max < -Extent.z)
				return false;

			Vector3 Normal = e0.Cross(e1);

			Vector3 vmin, vmax;
			float v;
			for (int i = 0; i <= 2; i++)
			{
				v = v0[i];
				if (Normal[i] > 0.0f)
				{
					vmin[i] = -Extent[i] - v;
					vmax[i] = Extent[i] - v;
				}
				else
				{
					vmin[i] = Extent[i] - v;
					vmax[i] = -Extent[i] - v;
				}
			}

			if (Normal.Dot(vmin) > 0.0f)
				return false;

			if (Normal.Dot(vmax) < 0.0f)
				return false;

			return true;
		}

		bool IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
		{
			return IntersectAABB(v0, v1, v2, Bmin, Bmax);
		}

		bool IntersectSphere(const Vector3& Center, float Radius) const
		{
			// Find point P on triangle ABC closest to sphere center
			Vector3 p = ClosestPointOnTriangleToPoint(Center, v0, v1, v2);

			// Sphere and triangle intersect if the (squared) distance from sphere
			// center to point p is less than the (squared) sphere radius
			Vector3 v = p - Center;
			return DotProduct(v, v) <= Radius * Radius;
		}

		bool IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const
		{
			Capsule3 capsule(X0, X1, Radius);
			return capsule.IntersectTriangle(v0, v1, v2);
		}

		bool IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
		{
			const float Tolerance = 1e-6f;
			const Triangle3& Triangle0 = *this;
			const Triangle3 Triangle1(A, B, C);

			Vector3 E0[3];
			E0[0] = Triangle0.v1 - Triangle0.v0;
			E0[1] = Triangle0.v2 - Triangle0.v1;
			E0[2] = Triangle0.v0 - Triangle0.v2;

			Vector3 N0 = UnitCrossProduct(E0[0], E0[1]);

			float N0dT0V0 = N0.Dot(Triangle0.v0);
			float min1, max1;
			Triangle1.ProjectOntoAxis(N0, min1, max1);
			if (N0dT0V0 < min1 - Tolerance || N0dT0V0 > max1 + Tolerance) {
				return false;
			}

			Vector3 E1[3];
			E1[0] = Triangle1.v1 - Triangle1.v0;
			E1[1] = Triangle1.v2 - Triangle1.v1;
			E1[2] = Triangle1.v0 - Triangle1.v2;

			Vector3 N1 = UnitCrossProduct(E1[0], E1[1]);

			Vector3 dir;
			float min0, max0;
			int i0, i1;

			Vector3 N0xN1 = UnitCrossProduct(N0, N1);
			if (N0xN1.Dot(N0xN1) >= Tolerance)
			{
				float N1dT1V0 = N1.Dot(Triangle1.v0);
				Triangle0.ProjectOntoAxis(N1, min0, max0);
				if (N1dT1V0 < min0 - Tolerance || N1dT1V0 > max0 + Tolerance)
				{
					return false;
				}

				for (i1 = 0; i1 < 3; ++i1)
				{
					for (i0 = 0; i0 < 3; ++i0)
					{
						dir = UnitCrossProduct(E0[i0], E1[i1]);
						Triangle0.ProjectOntoAxis(dir, min0, max0);
						Triangle1.ProjectOntoAxis(dir, min1, max1);
						if (max0 < min1 - Tolerance || max1 < min0 - Tolerance)
						{
							return false;
						}
					}
				}

			}
			else
			{
				for (i0 = 0; i0 < 3; ++i0)
				{
					dir = UnitCrossProduct(N0, E0[i0]);
					Triangle0.ProjectOntoAxis(dir, min0, max0);
					Triangle1.ProjectOntoAxis(dir, min1, max1);
					if (max0 < min1 - Tolerance || max1 < min0 - Tolerance)
					{
						return false;
					}
				}

				for (i1 = 0; i1 < 3; ++i1)
				{
					dir = UnitCrossProduct(N1, E1[i1]);
					Triangle0.ProjectOntoAxis(dir, min0, max0);
					Triangle1.ProjectOntoAxis(dir, min1, max1);
					if (max0 < min1 - Tolerance || max1 < min0 - Tolerance)
					{
						return false;
					}
				}
			}

			return true;
		}

		struct Triangle3IntersectionResult
		{
			int				Quantity{ 0 };
			Vector3			Points[6];
			IntersectionType Type{ IntersectionType::Empty };
		};

		bool IntersectTriangle(const Triangle3& triangle1, Triangle3IntersectionResult& Result) const
		{
			const Triangle3& triangle0 = *this;
			Plane3 Plane0(triangle0.v0, triangle0.v1, triangle0.v2);

			if (Plane0.Normal == Vector3::Zero())
			{
				Plane3 Plane1(triangle1.v0, triangle1.v1, triangle1.v2);
				if (Plane1.Normal != Vector3::Zero())
				{
					return triangle1.IntersectTriangle(triangle0, Result);
				}
				else
				{
					return false;
				}
			}

			int pos1, neg1, zero1;
			int sign1[3];
			float dist1[3];
			triangle1.CalculateRelationsToPlane(Plane0, dist1, sign1, pos1, neg1, zero1);

			if (pos1 == 3 || neg1 == 3)
			{
				return false;
			}

			if (zero1 == 3)
			{
				return false;
			}

			if (pos1 == 0 || neg1 == 0)
			{
				if (zero1 == 2)
				{
					for (int i = 0; i < 3; ++i)
					{
						if (sign1[i] != 0)
						{
							const int iM = (i + 2) % 3;
							const int iP = (i + 1) % 3;
							return triangle0.IntersectSegment_Coplanar(Plane0, triangle1[iM], triangle1[iP], Result);
						}
					}
				}
				else
				{
					for (int i = 0; i < 3; ++i)
					{
						if (sign1[i] == 0)
						{
							if (triangle0.ContainsPoint(triangle1[i]))
							{
								Result.Type = IntersectionType::Point;
								Result.Quantity = 1;
								Result.Points[0] = triangle1[i];
								return true;
							}
							return false;
						}
					}
				}
			}

			float t;
			Vector3 intr0, intr1;
			if (zero1 == 0)
			{
				int iSign = (pos1 == 1 ? +1 : -1);
				for (int i = 0; i < 3; ++i)
				{
					if (sign1[i] == iSign)
					{
						const int iM = (i + 2) % 3;
						const int iP = (i + 1) % 3;
						t = dist1[i] / (dist1[i] - dist1[iM]);
						intr0 = triangle1[i] + t * (triangle1[iM] - triangle1[i]);
						t = dist1[i] / (dist1[i] - dist1[iP]);
						intr1 = triangle1[i] + t * (triangle1[iP] - triangle1[i]);
						return triangle0.IntersectSegment_Coplanar(Plane0, intr0, intr1, Result);
					}
				}
			}

			for (int i = 0; i < 3; ++i)
			{
				if (sign1[i] == 0)
				{
					const int iM = (i + 2) % 3;
					const int iP = (i + 1) % 3;
					t = dist1[iM] / (dist1[iM] - dist1[iP]);
					intr0 = triangle1[iM] + t * (triangle1[iP] - triangle1[iM]);
					return triangle0.IntersectSegment_Coplanar(Plane0, triangle1[i], intr0, Result);
				}
			}

			// assert(false);
			return false;
		}

		void ProjectOntoAxis(const Vector3& axis, float& fmin, float& fmax) const
		{
			float dot0 = axis.Dot(v0);
			float dot1 = axis.Dot(v1);
			float dot2 = axis.Dot(v2);

			fmin = dot0;
			fmax = fmin;

			if (dot1 < fmin)
			{
				fmin = dot1;
			}
			else if (dot1 > fmax)
			{
				fmax = dot1;
			}

			if (dot2 < fmin)
			{
				fmin = dot2;
			}
			else if (dot2 > fmax)
			{
				fmax = dot2;
			}
		}

		static Vector3 ClosestPointOnTriangleToPoint(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C)
		{
			unsigned char mask;
			return ClosestPointOnTriangleToPointEx(Point, A, B, C, mask);
		}

		static float SqrDistancePointToTriangle(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C)
		{
			Vector3 Closest = ClosestPointOnTriangleToPoint(Point, A, B, C);
			return (Closest - Point).SquareLength();
		}

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

		struct BaryCentricQueryResult
		{
			float	SqrDistance;
			Vector3	ClosestPoint;
			Vector3	BaryCoords;
		};

		BaryCentricQueryResult BarycentricCoodsEx(const Vector3 &Point) const
		{
			Vector3 diff = Point - v0;
			Vector3 edge0 = v1 - v0;
			Vector3 edge1 = v2 - v0;
			float a00 = edge0.SquareLength();
			float a01 = edge0.Dot(edge1);
			float a11 = edge1.SquareLength();
			float b0 = -diff.Dot(edge0);
			float b1 = -diff.Dot(edge1);

			float f00 = b0;
			float f10 = b0 + a00;
			float f01 = b0 + a01;

			Vector2 p0, p1, p;
			float dt1, h0, h1;

			if (f00 >= 0.0f)
			{
				if (f01 >= 0.0f)
				{
					p = GetMinEdge02(a11, b1);
				}
				else
				{
					p0[0] = 0.0f;
					p0[1] = f00 / (f00 - f01);
					p1[0] = f01 / (f01 - f10);
					p1[1] = 1.0f - p1[0];
					dt1 = p1[1] - p0[1];
					h0 = dt1 * (a11 * p0[1] + b1);
					if (h0 >= 0.0f)
					{
						p = GetMinEdge02(a11, b1);
					}
					else
					{
						h1 = dt1 * (a01 * p1[0] + a11 * p1[1] + b1);
						if (h1 <= 0.0f)
						{
							p = GetMinEdge12(a01, a11, b1, f10, f01);
						}
						else
						{
							p = GetMinInterior(p0, h0, p1, h1);
						}
					}
				}
			}
			else if (f01 <= 0.0f)
			{
				if (f10 <= 0.0f)
				{
					p = GetMinEdge12(a01, a11, b1, f10, f01);
				}
				else
				{
					p0[0] = f00 / (f00 - f10);
					p0[1] = 0.0f;
					p1[0] = f01 / (f01 - f10);
					p1[1] = 1.0f - p1[0];
					h0 = p1[1] * (a01 * p0[0] + b1);
					if (h0 >= 0.0f)
					{
						p = p0;
					}
					else
					{
						h1 = p1[1] * (a01 * p1[0] + a11 * p1[1] + b1);
						if (h1 <= 0.0f)
						{
							p = GetMinEdge12(a01, a11, b1, f10, f01);
						}
						else
						{
							p = GetMinInterior(p0, h0, p1, h1);
						}
					}
				}
			}
			else if (f10 <= 0.0f)
			{
				p0[0] = 0.0f;
				p0[1] = f00 / (f00 - f01);
				p1[0] = f01 / (f01 - f10);
				p1[1] = 1.0f - p1[0];
				dt1 = p1[1] - p0[1];
				h0 = dt1 * (a11 * p0[1] + b1);
				if (h0 >= 0.0f)
				{
					p = GetMinEdge02(a11, b1);
				}
				else
				{
					h1 = dt1 * (a01 * p1[0] + a11 * p1[1] + b1);
					if (h1 <= 0.0f)
					{
						p = GetMinEdge12(a01, a11, b1, f10, f01);
					}
					else
					{
						p = GetMinInterior(p0, h0, p1, h1);
					}
				}
			}
			else
			{
				p0[0] = f00 / (f00 - f10);
				p0[1] = 0.0f;
				p1[0] = 0.0f;
				p1[1] = f00 / (f00 - f01);
				h0 = p1[1] * (a01 * p0[0] + b1);
				if (h0 >= 0.0f)
				{
					p = p0;
				}
				else
				{
					h1 = p1[1] * (a11 * p1[1] + b1);
					if (h1 <= 0.0f)
					{
						p = GetMinEdge02(a11, b1);
					}
					else
					{
						p = GetMinInterior(p0, h0, p1, h1);
					}
				}
			}

			BaryCentricQueryResult info;
			info.BaryCoords = Vector3(1.0f - p[0] - p[1], p[0], p[1]);
			info.ClosestPoint = v0 + p[0] * edge0 + p[1] * edge1;
			info.SqrDistance = (Point - info.ClosestPoint).SquareLength();
			return info;
		}

		void CalculateRelationsToPlane(const Plane3& plane, float distance[3], int sign[3], int& positive, int& negative, int& zero) const
		{
			const float Tolerance = 1e-6f;
			const Triangle3& triangle = *this;

			positive = 0;
			negative = 0;
			zero = 0;
			for (int i = 0; i < 3; ++i)
			{
				distance[i] = plane.SignedDistanceToPoint(triangle[i]);
				if (distance[i] > Tolerance)
				{
					sign[i] = 1;
					positive++;
				}
				else if (distance[i] < -Tolerance)
				{
					sign[i] = -1;
					negative++;
				}
				else
				{
					sign[i] = 0;
					zero++;
				}
			}
		}

		bool ContainsPoint(const Vector3& point) const
		{
			Vector3 Normal = GetNormal(v0, v1, v2);

			Vector3 u0, u1;
			Normal.GetPerpVectors(u0, u1);

			Vector3 pv0 = point - v0;
			Vector3 v10 = v1 - v0;
			Vector3 v20 = v2 - v0;
			Vector2 proj(u0.Dot(pv0), u1.Dot(pv0));
			Triangle2 pt(Vector2::Zero(), Vector2(u0.Dot(v10), u1.Dot(v10)), Vector2(u0.Dot(v20), u1.Dot(v20)));
			if (pt.IsInside(proj) <= 0)
			{
				return true;
			}

			return false;
		}

	private:
		// Assume end0 and end1 on opposite side of the plane(triangle), otherwise the result is incorrect
		bool IntersectSegment_Coplanar(const Plane3& plane, const Vector3& end0, const Vector3& end1, Triangle3IntersectionResult& Result) const
		{
			Result.Quantity = IntersectSegment_Coplanar(plane, end0, end1, Result.Points);
			if (Result.Quantity > 0)
			{
				Result.Type = Result.Quantity == 2 ? IntersectionType::Segment : IntersectionType::Point;
				return true;
			}
			else
			{
				Result.Type = IntersectionType::Empty;
				return false;
			}
		}

		// Assume end0 and end1 on opposite side of the plane(triangle), otherwise the result is incorrect
		int IntersectSegment_Coplanar(const Plane3& plane, const Vector3& end0, const Vector3& end1, Vector3* OutPts) const
		{
			const Triangle3 triangle = *this;

			const float Tolerance = 1e-6f;
			int max_axis = 0;
			float max_val = fabsf(plane.Normal.x);
			float val = fabsf(plane.Normal.y);
			if (val > max_val)
			{
				max_axis = 1;
				max_val = val;
			}
			val = fabsf(plane.Normal.z);
			if (val > max_val)
			{
				max_axis = 2;
			}

			Triangle2 proj_tri;
			Vector2 proj0, proj1;

			if (max_axis == 0)
			{
				// Project onto yz-plane.
				for (int i = 0; i < 3; ++i)
				{
					proj_tri[i] = Vector2(triangle[i].y, triangle[i].z);
				}
				proj0.x = end0.y;
				proj0.y = end0.z;
				proj1.x = end1.y;
				proj1.y = end1.z;
			}
			else if (max_axis == 1)
			{
				// Project onto xz-plane.
				for (int i = 0; i < 3; ++i)
				{
					proj_tri[i] = Vector2(triangle[i].x, triangle[i].z);
				}
				proj0.x = end0.x;
				proj0.y = end0.z;
				proj1.x = end1.x;
				proj1.y = end1.z;
			}
			else
			{
				// Project onto xy-plane.
				for (int i = 0; i < 3; ++i)
				{
					proj_tri[i] = Vector2(triangle[i].x, triangle[i].y);
				}
				proj0.x = end0.x;
				proj0.y = end0.y;
				proj1.x = end1.x;
				proj1.y = end1.y;
			}

			Triangle2::Segment2IntersectionResult Result;
			if (!proj_tri.IntersectSegment(proj0, proj1, Result))
			{
				return 0;
			}

			int Quantity = 0;

			Vector2 intr[2];
			if (Result.Type == IntersectionType::Segment)
			{
				Quantity = 2;
				intr[0] = Result.Point0;
				intr[1] = Result.Point1;
			}
			else
			{
				Quantity = 1;
				intr[0] = Result.Point0;
			}

			// Unproject the segment of intersection.
			if (max_axis == 0)
			{
				float inv = (1.0f) / plane.Normal.x;
				for (int i = 0; i < Quantity; ++i)
				{
					float y = intr[i].x;
					float z = intr[i].y;
					float x = inv * (plane.MinusD() - plane.Normal.y * y - plane.Normal.z * z);
					OutPts[i] = Vector3(x, y, z);
				}
			}
			else if (max_axis == 1)
			{
				float inv = (1.0f) / plane.Normal.y;
				for (int i = 0; i < Quantity; ++i)
				{
					float x = intr[i].x;
					float z = intr[i].y;
					float y = inv * (plane.MinusD() - plane.Normal.x * x - plane.Normal.z * z);
					OutPts[i] = Vector3(x, y, z);
				}
			}
			else
			{
				float inv = (1.0f) / plane.Normal.z;
				for (int i = 0; i < Quantity; ++i)
				{
					float x = intr[i].x;
					float y = intr[i].y;
					float z = inv * (plane.MinusD() - plane.Normal.x * x - plane.Normal.y * y);
					OutPts[i] = Vector3(x, y, z);
				}
			}

			return Quantity;
		}

		static Vector3 ClosestPointOnEdge(const Vector3& P0, const Vector3& P1, const Vector3& Point)
		{
			const Vector3 V1 = P1 - P0;
			const Vector3 V2 = Point - P0;

			const float dp1 = V2.Dot(V1);
			if (dp1 <= 0)
			{
				return P0;
			}

			const float dp2 = V1.Dot(V1);
			if (dp2 <= dp1)
			{
				return P1;
			}

			return P0 + V1 * (dp2 / dp2);
		}

		static Vector3 ClosestPointOnTriangleToPointEx(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C, unsigned char& mask)
		{
			const Vector3 BA = A - B;
			const Vector3 AC = C - A;
			const Vector3 CB = B - C;
			const Vector3 Normal = BA.Cross(CB);

			const Vector3 V[3] = { B, A, C };
			const Vector3 N[3] = { Normal.Cross(BA), Normal.Cross(AC), Normal.Cross(CB) };

			mask = 0;
			for (int i = 0; i < 3; ++i)
			{
				if ((Point - V[i]).Dot(N[i]) > 0.0f)
				{
					mask += (1 << i);
				}
			}

			if (mask == 0b0000)
			{
				float signedDist = (Point - A).Dot(Normal);
				return Point - signedDist * Normal;
			}
			else if (mask == 0b0001)
			{
				return ClosestPointOnEdge(B, A, Point);
			}
			else if (mask == 0b0010)
			{
				return ClosestPointOnEdge(A, C, Point);
			}
			else if (mask == 0b0011)
			{
				return A;
			}
			else if (mask == 0b0100)
			{
				return ClosestPointOnEdge(C, B, Point);
			}
			else if (mask == 0b0101)
			{
				return B;
			}
			else if (mask == 0b0110)
			{
				return C;
			}

			// assert(false);		// Should never comes here
			return A;
		}

		static Vector2 GetMinEdge02(float a11, float b1)
		{
			Vector2 p;
			p[0] = 0.0f;
			if (b1 >= 0.0f)
			{
				p[1] = 0.0f;
			}
			else if (a11 + b1 <= 0.0f)
			{
				p[1] = 1.0f;
			}
			else
			{
				p[1] = -b1 / a11;
			}
			return p;
		}

		static Vector2 GetMinEdge12(float a01, float a11, float b1, float f10, float f01)
		{
			Vector2 p;
			float h0 = a01 + b1 - f10;
			if (h0 >= 0.0f)
			{
				p[1] = 0.0f;
			}
			else
			{
				float h1 = a11 + b1 - f01;
				if (h1 <= 0.0f)
				{
					p[1] = 1.0f;
				}
				else
				{
					p[1] = h0 / (h0 - h1);
				}
			}
			p[0] = 1.0f - p[1];
			return p;
		}

		static Vector2 GetMinInterior(const Vector2 p0, float h0, const Vector2& p1, float h1)
		{
			float z = h0 / (h0 - h1);
			return (1.0f - z) * p0 + z * p1;
		}
	};
}