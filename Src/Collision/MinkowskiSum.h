#pragma once

#include "../Maths/Vector3.h"

class MinkowskiSum
{
public:
	virtual Vector3 Center() const = 0;
	virtual Vector3 Support(const Vector3& Dir) const = 0;
};
