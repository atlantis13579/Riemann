
#pragma once

#include <math.h>
#include "../Maths/Matrix3d.h"

class Capsule
{
public:
	float Radius;
	float Height;

public:
	Capsule() {}

	Capsule(float InRadius, float InHeight)
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
		return (float)M_PI * Radius * Radius * (Height + 4.0f * Radius / 3.0f);
	}

	Matrix3d GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Radius, Height, Mass);
	}

	static Matrix3d GetInertiaTensor(float Radius, float Height, float Mass)
	{
		// https://www.wolframalpha.com/input/?i=capsule&assumption=%7B%22C%22,+%22capsule%22%7D+-%3E+%7B%22Solid%22%7D
		float R = Radius;
		float H = Height;
		float RR = R * R;
		float HH = H * H;

		// (5H^3 + 20*H^2R + 45HR^2 + 32R^3) / (60H + 80R)
		float Diag12 = Mass * (5.0f * HH * H + 20.0f * HH * R + 45.0f * H * RR + 32.0f * RR * R) / (60.0f * H + 80.0f * R);
		// (R^2 * (15H + 16R) / (30H +40R))
		float Diag3 = Mass * (RR * (15.0f * H + 16.0f * R)) / (30.0f * H + 40.0f * R);

		return Matrix3d(Diag12, Diag12, Diag3);
	}
};