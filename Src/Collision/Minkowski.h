#pragma once

#include "../Maths/Vector3d.h"

class Geometry;

class Minkowski
{
public:
	Minkowski() {}
	Minkowski(Geometry* _g1, Geometry* _g2)
	{
		Geom1 = _g1;
		Geom2 = _g2;
	}

	Geometry* Geom1;
	Geometry* Geom2;

	Vector3d Support1(const Vector3d& Dir);
	Vector3d Support2(const Vector3d& Dir);
	Vector3d Support(const Vector3d& Dir);
};