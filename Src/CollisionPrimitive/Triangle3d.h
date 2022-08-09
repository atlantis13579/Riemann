
#pragma once

#include <assert.h>
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

	Vector3		GetNormal() const
	{
		return GetNormal(A, B, C);
	}

	static Vector3 GetNormal(const Vector3& A, const Vector3& B, const Vector3& C)
	{
		Vector3 cross = (B - A).Cross(C - A);
		return cross.Unit();
	}

	bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		return RayIntersectTriangle(Origin, Direction, A, B, C, t);
	}

	bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
	{
		return IntersectAABB(A, B, C, Bmin, Bmax);
	}

	// By Tomas Akenine-Moller
	// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
	// -----
	static bool		IntersectAABB(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& Bmin, const Vector3& Bmax)
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

	static Vector3 ClosestPointOnTriangle(const Vector3 &Point, const Vector3 &A, const Vector3 &B, const Vector3 &C, unsigned char &mask)
	{
		const Vector3 BA = A - B;
		const Vector3 AC = C - A;
		const Vector3 CB = B - C;
		const Vector3 Normal = BA.Cross(CB);
		
		const Vector3 V[3] = {B, A, C};
		const Vector3 N[3] = {Normal.Cross(BA), Normal.Cross(AC), Normal.Cross(CB)};
		
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
			return _ClosestPointOnEdge(B, A, Point);
		}
		else if (mask == 0b0010)
		{
			return _ClosestPointOnEdge(A, C, Point);
		}
		else if (mask == 0b0011)
		{
			return A;
		}
		else if (mask == 0b0100)
		{
			return _ClosestPointOnEdge(C, B, Point);
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
	
	static float	SqrDistancePointToTriangle(const Vector3 &Point, const Vector3 &A, const Vector3 &B, const Vector3 &C)
	{
		unsigned char mask = 0;
		Vector3 Closest = ClosestPointOnTriangle(Point, A, B, C, mask);
		return (Closest - Point).SquareLength();
	}
	
	float			SqrDistanceToPoint(const Vector3 &Point) const
	{
		return Triangle3d::SqrDistancePointToTriangle(Point, A, B, C);
	}
	
	bool			IntersectSphere(const Vector3& Center, float Radius) const
	{
		return SqrDistanceToPoint(Center) <= Radius * Radius;
	}

	// Moller CTrumbore intersection algorithm
	static bool		RayIntersectTriangle(const Vector3& Origin,
										const Vector3& Direction,
										const Vector3& A,
										const Vector3& B,
										const Vector3& C,
										float* t)
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
	
	Vector3		BaryCentric2D(const Vector3& Point)
	{
		float a = ((B.y - C.y) * (Point.x - C.x) + (C.x - B.x) * (Point.y - C.y)) / ((B.y - C.y) * (A.x - C.x) + (C.x - B.x) * (A.y - C.y));
		float b = ((C.y - A.y) * (Point.x - C.x) + (A.x - C.x) * (Point.y - C.y)) / ((B.y - C.y) * (A.x - C.x) + (C.x - B.x) * (A.y - C.y));
		return Vector3(a, b, 1.0f - a - b);
	}
	
	static float 	TriangleArea3D(const Vector3& A, const Vector3& B, const Vector3& C)
	{
		float cx = (B.y - A.y) * (C.z - A.z) - (C.y - A.y) * (B.z - A.z);
		float cy = (B.z - A.z) * (C.x - A.x) - (C.z - A.z) * (B.x - A.x);
		float cz = (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y);
		return 0.5f * sqrtf(cx * cx + cy * cy + cz * cz);
	}
	
	float			CalcArea() const
	{
		return Triangle3d::TriangleArea3D(A, B, C);
	}

	Box3d			GetBoundingVolume() const
	{
		Box3d box(A, A);
		box.Encapsulate(B, C);
		return box;
	}

	Matrix3		GetInertiaTensor(float Mass) const
	{
		// TODO
		return Matrix3(1, 1, 1);
	}

	Vector3		GetSupport(const Vector3& Direction) const
	{
		return GetSupport(A, B, C, Direction);
	}

	static Vector3 GetSupport(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& Direction)
	{
		float dpa = DotProduct(A, Direction);
		float dpb = DotProduct(B, Direction);
		float dpc = DotProduct(C, Direction);

		if (dpa >= dpb && dpa >= dpc)
		{
			return A;
		}
		else if (dpb >= dpa && dpb >= dpc)
		{
			return B;
		}

		return C;
	}

	int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
	{
		*FacePoints = Triangle3d::GetSupport(A, B, C, Direction);
		return 1;
	}

	void			GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
	{
		Vertices = std::vector<Vector3>({ A , B, C });
		Vector3 Nor = GetNormal();
		Normals = std::vector<Vector3>({ Nor, Nor , Nor });
		Indices = std::vector<uint16_t>({ 0,1,2, 2,3,0 });
	}

	void			GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
	{
		Vertices = std::vector<Vector3>({ A , B, C });
		Indices = std::vector<uint16_t>({ 0,1, 1,2, 2,0 });
	}
	
private:
	static Vector3 _ClosestPointOnEdge(const Vector3& P0, const Vector3& P1, const Vector3 &Point)
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
	
};



