#pragma once

#include "Point2d.h"

enum class ShapeType2d
{
	UNKNOWN = 0,
	RECT,
	CIRCLE,
	CONVEX,
};

class Shape2d
{
public:
	virtual Point2 Support(const Point2& Dir) const = 0;
};