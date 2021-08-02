
#pragma once

#include <math.h>
#include "../Maths/Matrix3d.h"
#include "ShapeType.h"

class Cylinder3d
{
public:
	float		Radius;
	float		Height;
	Vector3d	X0, X1;

public:
	Cylinder3d() {}

	Cylinder3d(float _Radius, const Vector3d& _X0, const Vector3d& _X1)
	{
		Radius = _Radius;
		Height = (_X0 - _X1).Length();
		X0 = _X0;
		X1 = _X1;
	}

	static constexpr ShapeType	StaticType()
	{
		return ShapeType::CYLINDER;
	}

public:

	inline float GetHeight() const
	{
		return Height;
	}

	float GetVolume() const
	{
		return GetVolume(Radius, Height);
	}

	static float GetVolume(float Radius, float Height)
	{
		return (float)M_PI * Radius * Radius * Height;
	}

	Matrix3d GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Radius, Height, Mass);
	}

	static Matrix3d GetInertiaTensor(float Radius, float Height, float Mass)
	{
		// https://www.wolframalpha.com/input/?i=cylinder
		float RR = Radius * Radius;
		float Diag12 = Mass / 12.0f * (3.0f * RR + Height * Height);
		float Diag3 = Mass / 2.0f * RR;
		return Matrix3d(Diag12, Diag12, Diag3);
	}

	void	GetMesh(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3d>& Normals)
	{

	}

	void	GetWireframe(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices)
	{

	}
};