#pragma once

#include <vector>
#include "../Maths/Vector3.h"

namespace Geometry
{
	bool SimplifyMesh(const Vector3* pv, const Vector3i* pi, int nv, int nt, float rate, std::vector<Vector3>& new_v, std::vector<int>& new_i);
}