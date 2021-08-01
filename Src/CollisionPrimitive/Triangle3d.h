
#pragma once

#include "../Maths/Vector3d.h"

class Triangle3d
{
public:
	Vector3d A, B, C;

public:
	Triangle3d(const Vector3d& InA, const Vector3d& InB, const Vector3d& InC)
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

	Vector3d GetNormal() const
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

	// Moller CTrumbore intersection algorithm
	static bool RayIntersectTriangle(const Vector3d& Origin,
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
		if (det > -kEpsilonTri && det < kEpsilonTri) {
			return false;
		}
		const float inv_det = 1.f / det;

		//calculate distance from V1 to ray origin
		const Vector3d T = Origin - A;

		//Calculate u parameter and test bound
		const float u = T.Dot(P) * inv_det;
		//The intersection lies outside of the triangle
		if (u < 0.f || u > 1.f) {
			return false;
		}

		//Prepare to test v parameter
		const Vector3d Q = T.Cross(e1);

		//Calculate V parameter and test bound
		const float v = Dir.Dot(Q) * inv_det;
		//The intersection lies outside of the triangle
		if (v < 0.f || u + v  > 1.f) {
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

	Box3d	GetBoundingVolume() const
	{
		Box3d box(A, A);
		box.Grow(B);
		box.Grow(C);
		return box;
	}

	Matrix3d GetInertiaTensor(float Mass) const
	{
		// TODO
		return Matrix3d(1, 1, 1);
	}

	Vector3d GetSupport(const Vector3d& dir) const
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

	void	GetMesh(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3d>& Normals)
	{
		Vertices = { A , B, C };
		Vector3d Nor = GetNormal();
		Normals = { Nor, Nor , Nor };
		Indices = { 0,1,2, 2,3,0 };
	}

	void	GetWireframe(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices)
	{
		Vertices = { A , B, C };
		Indices = { 0,1, 1,2, 2,0 };
	}
};



