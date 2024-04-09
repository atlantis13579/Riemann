
#include <assert.h>
#include "Triangle3d.h"
#include "Capsule3d.h"
#include "Plane3d.h"
#include "Sphere3d.h"
#include "Segment2d.h"
#include "Triangle2d.h"

namespace Riemann
{
	bool Triangle3d::IntersectPoint(const Vector3& Point) const
	{
		Vector3 bc = BaryCentric2D(Point);
		return bc.x >= 0 && bc.y >= 0.0f && bc.z >= 0.0f;
	}

	bool Triangle3d::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		return RayIntersectTriangle(Origin, Direction, v0, v1, v2, t);
	}

	// By Tomas Akenine-Moller
	// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
	// -----
	bool Triangle3d::IntersectAABB(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& Bmin, const Vector3& Bmax)
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

	bool Triangle3d::IntersectSegment(const Vector3& P0, const Vector3& P1) const
	{
		Vector3 dir = P0 - P1;
		Vector3 n = GetNormal();

		float d = dir.Dot(n);
		if (d <= 0.0f)
			return false;

		Vector3 ap = P0 - v0;
		float t = ap.Dot(n);
		if (t > d || t < 0.0f)
			return false;

		Vector3 e = dir.Cross(ap);
		float v = (v2 - v0).Dot(e);
		if (v < 0.0f || v > d)
			return false;
		float w = -(v1 - v0).Dot(e);
		if (w < 0.0f || v + w > d)
			return false;

		return true;
	}

	bool Triangle3d::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
	{
		return IntersectAABB(v0, v1, v2, Bmin, Bmax);
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

	// static
	Vector3 Triangle3d::ClosestPointOnTriangleToPoint(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C)
	{
		unsigned char mask;
		return ClosestPointOnTriangleToPointEx(Point, A, B, C, mask);
	}

	// static
	Vector3 Triangle3d::ClosestPointOnTriangleToPointEx(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C, unsigned char& mask)
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

		assert(false);		// Should never comes here
		return A;
	}

	float Triangle3d::SqrDistancePointToTriangle(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C)
	{
		Vector3 Closest = ClosestPointOnTriangleToPoint(Point, A, B, C);
		return (Closest - Point).SquareLength();
	}

	float Triangle3d::SqrDistanceToPoint(const Vector3& Point) const
	{
		return Triangle3d::SqrDistancePointToTriangle(Point, v0, v1, v2);
	}

	Vector3	Triangle3d::ClosestPointToPoint(const Vector3& Point) const
	{
		return Triangle3d::ClosestPointOnTriangleToPoint(Point, v0, v1, v2);
	}

	bool Triangle3d::IntersectSphere(const Vector3& Center, float Radius) const
	{
		// Find point P on triangle ABC closest to sphere center
		Vector3 p = ClosestPointOnTriangleToPoint(Center, v0, v1, v2);

		// Sphere and triangle intersect if the (squared) distance from sphere
		// center to point p is less than the (squared) sphere radius
		Vector3 v = p - Center;
		return DotProduct(v, v) <= Radius * Radius;
	}

	bool Triangle3d::IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const
	{
		Capsule3d capsule(X0, X1, Radius);
		return capsule.IntersectTriangle(v0, v1, v2);
	}

	static void ProjectOntoAxis(const Triangle3d& triangle, const Vector3& axis, float& fmin, float& fmax)
	{
		float dot0 = axis.Dot(triangle.v0);
		float dot1 = axis.Dot(triangle.v1);
		float dot2 = axis.Dot(triangle.v2);

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

	bool Triangle3d::IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
	{
		const float Tolerance = 1e-6f;
		const Triangle3d &Triangle0 = *this;
		const Triangle3d Triangle1(A, B, C);

		Vector3 E0[3];
		E0[0] = Triangle0.v1 - Triangle0.v0;
		E0[1] = Triangle0.v2 - Triangle0.v1;
		E0[2] = Triangle0.v0 - Triangle0.v2;

		Vector3 N0 = UnitCrossProduct(E0[0], E0[1]);

		float N0dT0V0 = N0.Dot(Triangle0.v0);
		float min1, max1;
		ProjectOntoAxis(Triangle1, N0, min1, max1);
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
			ProjectOntoAxis(Triangle0, N1, min0, max0);
			if (N1dT1V0 < min0 - Tolerance || N1dT1V0 > max0 + Tolerance)
			{
				return false;
			}

			for (i1 = 0; i1 < 3; ++i1)
			{
				for (i0 = 0; i0 < 3; ++i0)
				{
					dir = UnitCrossProduct(E0[i0], E1[i1]);
					ProjectOntoAxis(Triangle0, dir, min0, max0);
					ProjectOntoAxis(Triangle1, dir, min1, max1);
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
				ProjectOntoAxis(Triangle0, dir, min0, max0);
				ProjectOntoAxis(Triangle1, dir, min1, max1);
				if (max0 < min1 - Tolerance || max1 < min0 - Tolerance)
				{
					return false;
				}
			}

			for (i1 = 0; i1 < 3; ++i1)
			{
				dir = UnitCrossProduct(N1, E1[i1]);
				ProjectOntoAxis(Triangle0, dir, min0, max0);
				ProjectOntoAxis(Triangle1, dir, min1, max1);
				if (max0 < min1 - Tolerance || max1 < min0 - Tolerance)
				{
					return false;
				}
			}
		}

		return true;
	}

	// Moller CTrumbore intersection algorithm
	bool Triangle3d::RayIntersectTriangle(const Vector3& Origin, const Vector3& Direction, const Vector3& A, const Vector3& B, const Vector3& C, float* t)
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

	bool Triangle3d::IsColinear(const Vector3& a, const Vector3& b, const Vector3& c)
	{
		Vector3 n = (b - a).Cross(c - a);
		return n.SquareLength() < 1e-6f;
	}

	Vector3 Triangle3d::BaryCentric2D(const Vector3& Point) const
	{
		float a = ((v1.y - v2.y) * (Point.x - v2.x) + (v2.x - v1.x) * (Point.y - v2.y)) / ((v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y));
		float b = ((v2.y - v0.y) * (Point.x - v2.x) + (v0.x - v2.x) * (Point.y - v2.y)) / ((v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y));
		return Vector3(a, b, 1.0f - a - b);
	}

	inline float TriArea2D(float x1, float y1, float x2, float y2, float x3, float y3)
	{
		return (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2);
	}

	Vector3	Triangle3d::BaryCentric3D(const Vector3& Point) const
	{
		Vector3 v0 = v1 - v0, v1 = v2 - v0, v2 = Point - v0;
		float d00 = DotProduct(v0, v0);
		float d01 = DotProduct(v0, v1);
		float d11 = DotProduct(v1, v1);
		float d20 = DotProduct(v2, v0);
		float d21 = DotProduct(v2, v1);
		float denom = d00 * d11 - d01 * d01;
		float v = (d11 * d20 - d01 * d21) / denom;
		float w = (d00 * d21 - d01 * d20) / denom;
		float u = 1.0f - v - w;
		return Vector3(u, v, w);
	}

	float Triangle3d::TriangleArea3D(const Vector3& A, const Vector3& B, const Vector3& C)
	{
		float cx = (B.y - A.y) * (C.z - A.z) - (C.y - A.y) * (B.z - A.z);
		float cy = (B.z - A.z) * (C.x - A.x) - (C.z - A.z) * (B.x - A.x);
		float cz = (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y);
		return 0.5f * sqrtf(cx * cx + cy * cy + cz * cz);
	}

	static bool PointOutsideOfPlane(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c)
	{
		return (p - a).Dot((b - a).Cross(c - a)) >= 0.0f; // [AP AB AC] >= 0
	}

	Vector3 ClosestPtTetrahedronToPoint(const Vector3 & p, const Vector3 & a, const Vector3 & b, const Vector3 & c, const Vector3 & d)
	{
		Vector3 closestPt = p;
		float min_dist = FLT_MAX;
		if (PointOutsideOfPlane(p, a, b, c))
		{
			const Vector3 q = Triangle3d::ClosestPointOnTriangleToPoint(p, a, b, c);
			const float dist = DotProduct(q - p, q - p);
			if (dist < min_dist)
			{
				min_dist = dist;
				closestPt = q;
			}
		}

		if (PointOutsideOfPlane(p, a, c, d))
		{
			const Vector3 q = Triangle3d::ClosestPointOnTriangleToPoint(p, a, c, d);
			const float dist = DotProduct(q - p, q - p);
			if (dist < min_dist)
			{
				min_dist = dist;
				closestPt = q;
			}
		}

		if (PointOutsideOfPlane(p, a, d, b))
		{
			const Vector3 q = Triangle3d::ClosestPointOnTriangleToPoint(p, a, d, b);
			const float dist = DotProduct(q - p, q - p);
			if (dist < min_dist)
			{
				min_dist = dist;
				closestPt = q;
			}
		}

		if (PointOutsideOfPlane(p, b, d, c))
		{
			const Vector3 q = Triangle3d::ClosestPointOnTriangleToPoint(p, b, d, c);
			const float dist = DotProduct(q - p, q - p);
			if (dist < min_dist)
			{
				min_dist = dist;
				closestPt = q;
			}
		}
		return closestPt;
	}

	static void TrianglePlaneRelations(const Triangle3d& triangle, const Plane3d& plane,
		float distance[3], int sign[3], int& positive, int& negative, int& zero,
		const float Tolerance)
	{
		positive = 0;
		negative = 0;
		zero = 0;
		for (int i = 0; i < 3; ++i)
		{
			distance[i] = plane.SignedDistanceTo(triangle[i]);
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

	static bool ContainsPoint(const Triangle3d& triangle, const Vector3& plane_normal, float plane_d, const Vector3& point)
	{
		Plane3d plane(plane_normal, plane_d);

		Vector3 U0, U1;
		plane.Normal.GetPerpVectors(U0, U1);

		Vector3 PmV0 = point - triangle.v0;
		Vector3 V1mV0 = triangle.v1 - triangle.v0;
		Vector3 V2mV0 = triangle.v2 - triangle.v0;
		Vector2 ProjP(U0.Dot(PmV0), U1.Dot(PmV0));
		Triangle2d ProjT(Vector2::Zero(), Vector2(U0.Dot(V1mV0), U1.Dot(V1mV0)), Vector2(U0.Dot(V2mV0), U1.Dot(V2mV0)));
		if (ProjT.IsInside(ProjP) <= 0)
		{
			return true;
		}

		return false;
	}

	static Vector2 GetXY(const Vector3& v)
	{
		return Vector2(v.x, v.y);
	}

	static Vector2 GetXZ(const Vector3& v)
	{
		return Vector2(v.x, v.z);
	}

	static Vector2 GetYZ(const Vector3& v)
	{
		return Vector2(v.y, v.z);
	}

	static int IntersectTriangleWithCoplanarSegment(
		const Plane3d& plane, const Triangle3d& triangle, const Vector3& end0, const Vector3& end1,
		Vector3& OutA, Vector3& OutB, float Tolerance)
	{
		int maxNormal = 0;
		float fmax = fabsf(plane.Normal.x);
		float absMax = fabsf(plane.Normal.y);
		if (absMax > fmax)
		{
			maxNormal = 1;
			fmax = absMax;
		}
		absMax = fabsf(plane.Normal.z);
		if (absMax > fmax) {
			maxNormal = 2;
		}

		Triangle2d projTri;
		Vector2 projEnd0, projEnd1;
		int i;

		if (maxNormal == 0)
		{
			// Project onto yz-plane.
			for (i = 0; i < 3; ++i)
			{
				projTri[i] = GetYZ(triangle[i]);
			}
			projEnd0.x = end0.y;
			projEnd0.y = end0.z;
			projEnd1.x = end1.y;
			projEnd1.y = end1.z;
		}
		else if (maxNormal == 1)
		{
			// Project onto xz-plane.
			for (i = 0; i < 3; ++i)
			{
				projTri[i] = GetXZ(triangle[i]);
			}
			projEnd0.x = end0.x;
			projEnd0.y = end0.z;
			projEnd1.x = end1.x;
			projEnd1.y = end1.z;
		}
		else
		{
			// Project onto xy-plane.
			for (i = 0; i < 3; ++i)
			{
				projTri[i] = GetXY(triangle[i]);
			}
			projEnd0.x = end0.x;
			projEnd0.y = end0.y;
			projEnd1.x = end1.x;
			projEnd1.y = end1.y;
		}

		Segment2dTriangle2dIntersectionResult Result;
		Segment2d projSeg(projEnd0, projEnd1);
		
		if (!CalculateIntersectionSegment2dTriangle2d(projSeg, projTri, Result))
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

		Vector3* OutPts[2]{ &OutA, &OutB };

		// Unproject the segment of intersection.
		if (maxNormal == 0)
		{
			float invNX = ((float)1) / plane.Normal.x;
			for (i = 0; i < Quantity; ++i)
			{
				float y = intr[i].x;
				float z = intr[i].y;
				float x = invNX * (plane.MinusConstant() - plane.Normal.y * y - plane.Normal.z * z);
				*OutPts[i] = Vector3(x, y, z);
			}
		}
		else if (maxNormal == 1)
		{
			float invNY = ((float)1) / plane.Normal.y;
			for (i = 0; i < Quantity; ++i)
			{
				float x = intr[i].x;
				float z = intr[i].y;
				float y = invNY * (plane.MinusConstant() - plane.Normal.x * x - plane.Normal.z * z);
				*OutPts[i] = Vector3(x, y, z);
			}
		}
		else
		{
			float invNZ = ((float)1) / plane.Normal.z;
			for (i = 0; i < Quantity; ++i)
			{
				float x = intr[i].x;
				float y = intr[i].y;
				float z = invNZ * (plane.MinusConstant() - plane.Normal.x * x - plane.Normal.y * y);
				*OutPts[i] = Vector3(x, y, z);
			}
		}

		return Quantity;
	}

	bool IntersectsSegment(const Vector3& plane_normal, float plane_d, const Triangle3d& triangle, const Vector3& end0, const Vector3& end1, Triangle3dTriangle3dIntersectionResult& Result)
	{
		Plane3d plane(plane_normal, plane_d);

		Result.Quantity = IntersectTriangleWithCoplanarSegment(plane, triangle, end0, end1, Result.Points[0], Result.Points[1], 1e-6f);
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

	bool CalculateIntersectionTriangle3dTriangle3d(const Triangle3d& triangle0, const Triangle3d& triangle1, Triangle3dTriangle3dIntersectionResult& Result)
	{
		int i, iM, iP;

		Plane3d Plane0(triangle0.v0, triangle0.v1, triangle0.v2);

		if (Plane0.Normal == Vector3::Zero())
		{
			Plane3d Plane1(triangle1.v0, triangle1.v1, triangle1.v2);
			if (Plane1.Normal != Vector3::Zero())
			{
				return CalculateIntersectionTriangle3dTriangle3d(triangle1, triangle0, Result);
			}
			else
			{
				return false;
			}
		}

		int pos1, neg1, zero1;
		int sign1[3];
		float dist1[3];
		TrianglePlaneRelations(triangle1, Plane0, dist1, sign1, pos1, neg1, zero1, 1e-6f);

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
				for (i = 0; i < 3; ++i)
				{
					if (sign1[i] != 0)
					{
						iM = (i + 2) % 3;
						iP = (i + 1) % 3;
						return IntersectsSegment(Plane0.Normal, Plane0.D, triangle0, triangle1[iM], triangle1[iP], Result);
					}
				}
			}
			else
			{
				for (i = 0; i < 3; ++i)
				{
					if (sign1[i] == 0)
					{
						if (ContainsPoint(triangle0, Plane0.Normal, Plane0.D, triangle1[i]))
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
			for (i = 0; i < 3; ++i)
			{
				if (sign1[i] == iSign)
				{
					iM = (i + 2) % 3;
					iP = (i + 1) % 3;
					t = dist1[i] / (dist1[i] - dist1[iM]);
					intr0 = triangle1[i] + t * (triangle1[iM] - triangle1[i]);
					t = dist1[i] / (dist1[i] - dist1[iP]);
					intr1 = triangle1[i] + t * (triangle1[iP] - triangle1[i]);
					return IntersectsSegment(Plane0.Normal, Plane0.D, triangle0, intr0, intr1, Result);
				}
			}
		}

		for (i = 0; i < 3; ++i)
		{
			if (sign1[i] == 0)
			{
				iM = (i + 2) % 3;
				iP = (i + 1) % 3;
				t = dist1[iM] / (dist1[iM] - dist1[iP]);
				intr0 = triangle1[iM] + t * (triangle1[iP] - triangle1[iM]);
				return IntersectsSegment(Plane0.Normal, Plane0.D, triangle0, triangle1[i], intr0, Result);
			}
		}

		assert(false);
		return false;
	}
}