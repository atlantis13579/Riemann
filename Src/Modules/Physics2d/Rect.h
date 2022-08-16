#pragma once

#include "Shape2d.h"

class Rect : public Shape2d
{
public:
	Point2	Center;
	Point2	HalfExtent;
	float	Rot;

	Rect(const Point2& _c, const Point2& _h, float _rot)
	{
		Center = _c;
		HalfExtent = _h;
		Rot = _rot;
	}

public:
	virtual Point2 Support(const Point2& Dir) const override final
	{
		Point2 DirLocal = Dir.Rotate(-Rot);
		Point2 SupportLocal = Point2(
			DirLocal.x > 0 ? Center.x + HalfExtent.x : Center.x - HalfExtent.x,
			DirLocal.y > 0 ? Center.y + HalfExtent.y : Center.y - HalfExtent.y
		);
		return SupportLocal.Rotate(Rot);
	}
};