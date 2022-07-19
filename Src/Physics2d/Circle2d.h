#pragma once

#include <math.h>
#include "Shape2d.h"

class Circle2d : public Shape2d
{
public:
	Point2	Center;
	float	Radius;

	Circle2d(const Point2& _c, float _r)
	{
		Center = _c;
		Radius = _r;
	}

public:
	virtual Point2 Support(const Point2& Dir) const override final
	{
		float distSqr = Dir.SquareLength();
		if (distSqr <= 1e-6)
		{
			return Center;
		}
		Point2 Normalized = Dir / sqrtf(distSqr);
		return Center + Normalized * Radius;
	}
};