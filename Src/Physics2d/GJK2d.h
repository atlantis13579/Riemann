#pragma once

#include "Shape2d.h"

class Simplex2d
{

};

class GJK2d
{
public:
	Simplex2d simplex;

	bool Solve(const Shape2d *shape1, const Shape2d *shape2, const Point2& InitGuess)
	{
		return true;
	}
};