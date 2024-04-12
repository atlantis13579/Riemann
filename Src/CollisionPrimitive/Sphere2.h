#pragma once

#include <math.h>
#include "../Maths/Vector2.h"

class Sphere2
{
public:
	Vector2	Center;
	float	Radius;

	Sphere2(const Vector2& _c, float _r)
	{
		Center = _c;
		Radius = _r;
	}

public:
	Vector2 Support(const Vector2& Dir) const
	{
		float distSqr = Dir.SquareLength();
		if (distSqr <= 1e-6)
		{
			return Center;
		}
		Vector2 Normalized = Dir / sqrtf(distSqr);
		return Center + Normalized * Radius;
	}
};