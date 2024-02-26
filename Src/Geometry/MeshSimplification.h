#pragma once

#include <vector>
#include "../Maths/Vector3.h"

namespace Riemann
{
	bool SimplifyMesh(const Vector3* pv, const void* pi, int nv, int ni, bool is16bit, float rate, std::vector<Vector3>& new_v, std::vector<int>& new_i);
}