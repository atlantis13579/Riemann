
#pragma once

#include <stdint.h>

#include "../Maths/Vector3d.h"
#include "ShapeType.h"

class Triangle3d
{
public:
	Vector3d A, B, C;

public:
	Triangle3d()
	{
	}

	Triangle3d(const Vector3d& InA, const Vector3d& InB, const Vector3d& InC)
	{
		Init(InA, InB, InC);
	}

	static constexpr ShapeType	StaticType()
	{
		return ShapeType::TRIANGLE;
	}

	void			Init(const Vector3d& InA, const Vector3d& InB, const Vector3d& InC)
	{
		A = InA;
		B = InB;
		C = InC;
	}

	Vector3d operator[](int i)
	{
		return (&A)[i];
	}

	const Vector3d& operator[](int i) const
	{
		return (&A)[i];
	}

	Vector3d		GetNormal() const
	{
		return GetNormal(A, B, C);
	}

	static Vector3d GetNormal(const Vector3d& A, const Vector3d& B, const Vector3d& C)
	{
		Vector3d cross = (B - A).Cross(C - A);
		return cross.Unit();
	}

	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
	{
		return RayIntersectTriangle(Origin, Dir, A, B, C, t);
	}

	bool			IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const
	{
		return IntersectAABB(A, B, C, Bmin, Bmax);
	}

	// By Tomas Akenine-Moller
	// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
	// -----
	static bool		IntersectAABB(const Vector3d& A, const Vector3d& B, const Vector3d& C, const Vector3d& Bmin, const Vector3d& Bmax)
	{
		Vector3d Center = (Bmax + Bmin) * 0.5f;
		Vector3d Extent = Bmax - Center;

		float min, max, p0, p1, p2, rad, fex, fey, fez;

		Vector3d v0 = A - Center;
		Vector3d v1 = B - Center;
		Vector3d v2 = C - Center;

		Vector3d e0 = v1 - v0;
		Vector3d e1 = v2 - v1;
		Vector3d e2 = v0 - v2;

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

		Vector3d Normal = e0.Cross(e1);

		Vector3d vmin, vmax;
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

	// Moller CTrumbore intersection algorithm
	static bool		RayIntersectTriangle(const Vector3d& Origin,
										const Vector3d& Dir,
										const Vector3d& A,
										const Vector3d& B,
										const Vector3d& C,
										float* t)
	{
		const float kEpsilonTri = 0.0000001f;
		//Find vectors for two edges sharing V1
		const Vector3d e1 = B - A;
		const Vector3d e2 = C - A;
		//Begin calculating determinant - also used to calculate u parameter
		const Vector3d P = Dir.Cross(e2);
		//if determinant is near zero, ray lies in plane of triangle
		const float det = e1.Dot(P);
		//NOT CULLING
		if (det > -kEpsilonTri && det < kEpsilonTri)
		{
			return false;
		}
		const float inv_det = 1.f / det;

		//calculate distance from V1 to ray origin
		const Vector3d T = Origin - A;

		//Calculate u parameter and test bound
		const float u = T.Dot(P) * inv_det;
		//The intersection lies outside of the triangle
		if (u < 0.f || u > 1.f)
		{
			return false;
		}

		//Prepare to test v parameter
		const Vector3d Q = T.Cross(e1);

		//Calculate V parameter and test bound
		const float v = Dir.Dot(Q) * inv_det;
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

	Box3d			GetBoundingVolume() const
	{
		Box3d box(A, A);
		box.Grow(B);
		box.Grow(C);
		return box;
	}

	Matrix3d		GetInertiaTensor(float Mass) const
	{
		// TODO
		return Matrix3d(1, 1, 1);
	}

	Vector3d		GetSupport(const Vector3d& dir) const
	{
		return GetSupport(A, B, C, dir);
	}

	static Vector3d GetSupport(const Vector3d& A, const Vector3d& B, const Vector3d& C, const Vector3d& Dir)
	{
		float dpa = DotProduct(A, Dir);
		float dpb = DotProduct(B, Dir);
		float dpc = DotProduct(C, Dir);

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

	void			GetMesh(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3d>& Normals)
	{
		Vertices = std::vector<Vector3d>({ A , B, C });
		Vector3d Nor = GetNormal();
		Normals = std::vector<Vector3d>({ Nor, Nor , Nor });
		Indices = std::vector<uint16_t>({ 0,1,2, 2,3,0 });
	}

	void			GetWireframe(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices)
	{
		Vertices = std::vector<Vector3d>({ A , B, C });
		Indices = std::vector<uint16_t>({ 0,1, 1,2, 2,0 });
	}
};



