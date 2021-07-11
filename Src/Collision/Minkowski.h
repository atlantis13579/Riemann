#pragma once

#include "../Maths/Vector3d.h"

class MinkowskiSum
{
public:
	virtual Vector3d Support(const Vector3d& Dir) = 0;
};