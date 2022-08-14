#pragma once

#include <math.h>
#include <vector>

#include "ShapeType.h"
#include "../Maths/Box3d.h"

class Cylinder3d
{
public:
	float		Radius;
	float		Height;
	Vector3		X0, X1;

public:
	Cylinder3d() {}

	Cylinder3d(const Vector3& _X0, const Vector3& _X1, float _Radius)
	{
		Init(_X0, _X1, _Radius);
	}

	void Init(const Vector3& _X0, const Vector3& _X1, float _Radius)
	{
		Radius = _Radius;
		Height = (_X0 - _X1).Length();
		X0 = _X0;
		X1 = _X1;
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::CYLINDER;
	}

public:

	inline Vector3 GetCenter() const
	{
		return (X0 + X1) * 0.5f;
	}

	inline Vector3 GetAxis() const
	{
		return X1 - X0;
	}

	inline Vector3	GetUnitAxis() const
	{
		return (X1 - X0).Unit();
	}

	inline float GetHeight() const
	{
		return Height;
	}

	inline float GetHalfHeight() const
	{
		return Height * 0.5f;
	}

	float GetVolume() const
	{
		return GetVolume(Radius, Height);
	}

	static float GetVolume(float Radius, float Height)
	{
		return (float)M_PI * Radius * Radius * Height;
	}

	Box3d		GetBoundingVolume() const;

	Matrix3 GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Radius, Height, Mass);
	}

	static Matrix3 GetInertiaTensor(float Radius, float Height, float Mass)
	{
		// https://www.wolframalpha.com/input/?i=cylinder
		float RR = Radius * Radius;
		float Diag12 = Mass / 12.0f * (3.0f * RR + Height * Height);
		float Diag3 = Mass / 2.0f * RR;
		return Matrix3(Diag12, Diag12, Diag3);
	}
	
	bool		IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
	static bool IntersectRayInfinityLength(const Vector3& Origin, const Vector3& Direction, float Radius, float *t);
	bool		IntersectSegment(const Vector3& P0, const Vector3& P1) const;

	Vector3		GetSupport(const Vector3& Direction) const;
	int			GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const;

	void		GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);
	void		GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices);
};
