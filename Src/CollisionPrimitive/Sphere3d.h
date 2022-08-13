#pragma once

#include <math.h>
#include <stdint.h>
#include <vector>
#include "ShapeType.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Box3d.h"

class Sphere3d
{
public:
	Vector3 Center;
	float Radius;

public:
	Sphere3d()
	{
	}

	Sphere3d(const Vector3& _Center, float _Radius)
	{
		Center = _Center;
		Radius = _Radius;
	}
	
	Sphere3d(const Vector3& A);
	Sphere3d(const Vector3& A, const Vector3 &B);
	Sphere3d(const Vector3& A, const Vector3 &B, const Vector3 &C);
	Sphere3d(const Vector3& A, const Vector3 &B, const Vector3 &C, const Vector3 &D);

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::SPHERE;
	}

public:
	bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
	static bool		RayIntersectSphere(const Vector3& Origin, const Vector3& Direction, const Vector3& Center, float Radius, float* t);
	bool			IntersectPoint(const Vector3& Point) const;
	bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
	bool			IntersectSphere(const Vector3 &rCenter, float rRadius) const;
	bool			IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3 &C) const;
	static bool		SphereIntersectSphere(const Vector3& Center, float Radius, const Vector3& rCenter, float rRadius);

	static Box3d	CalcBoundingVolume(const Vector3& Center, float Radius)
	{
		Box3d Box;
		Box.BuildFromCenterAndExtent(Center, Vector3(Radius));
		return Box;
	}
	
	Box3d			GetBoundingVolume() const
	{
		return Sphere3d::CalcBoundingVolume(Center, Radius);
	}
	
	void			Enlarge(float d) { Radius += d; }
	Sphere3d& 		Encapsulate(const Vector3& p);
	
	static Sphere3d	ComputeBoundingSphere_MostSeparated(const Vector3 *points, int n);
	static Sphere3d	ComputeBoundingSphere_Eigen(const Vector3 *points, int n);
	static Sphere3d	ComputeBoundingSphere_RitterIteration(const Vector3 *points, int n);
	static Sphere3d	ComputeBoundingSphere_Welzl(const Vector3 *points, int n);
	
	float			GetVolume() const
	{
		const float FourThirdsPI = (4.0f / 3.0f) * (float)M_PI;
		return FourThirdsPI * Radius * Radius * Radius;
	}

	Vector3			GetCenterOfMass() const
	{
		return Center;
	}

	Matrix3		GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Radius, Mass);
	}

	static Matrix3 GetInertiaTensor(float Radius, float Mass)
	{
		const float Diagonal = 2.0f * Mass * Radius * Radius / 5.0f;
		return Matrix3(Diagonal, Diagonal, Diagonal);
	}

	Vector3			GetSupport(const Vector3& Direction) const;
	static Vector3	GetSupport(const Vector3& Center, float Radius, const Vector3& Direction);
	int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const;

	void			GetVertices(int stackCount, int sliceCount, std::vector<Vector3>* Vertices, std::vector<Vector3>* Normals);
	void			GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);
	void			GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices);
};
