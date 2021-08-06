#pragma once

#include "../Maths/Vector3d.h"

class Point3d 
{
public:
	Vector3d	P;

	Point3d(const Vector3d &_p)
	{
		P = _p;
	}
};