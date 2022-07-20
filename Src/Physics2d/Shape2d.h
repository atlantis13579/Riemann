#pragma once

#include "Point2d.h"

class Shape2d
{
public:
	virtual Point2 Support(const Point2& Dir) const = 0;
};