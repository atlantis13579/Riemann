#pragma once

#include "../Maths/Vector3d.h"

class MinkowskiSum
{
public:
	virtual Vector3d Center() const = 0;
	virtual Vector3d Support(const Vector3d& Dir) const = 0;
};
