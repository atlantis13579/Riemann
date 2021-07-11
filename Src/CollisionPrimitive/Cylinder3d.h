
#pragma once

#include <math.h>
#include "../Maths/Matrix3d.h"

class Cylinder3d
{
public:
	float Radius;
	float Height;

public:
	Cylinder3d() {}

	Cylinder3d(float InRadius, float InHeight)
	{
		Radius = InRadius;
		Height = InHeight;
	}

public:
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
};