#pragma once

#include <vector>
#include "../Maths/Vector3.h"

namespace Riemann
{
	struct SimplificationConfig
	{
		float	rate = 1.0f;
		int		faces = -1;
		float	t = 0.01f;
	};

	bool SimplifyMesh(const Vector3* pv, const void* pi, int nv, int nt, bool is16bit, const SimplificationConfig& cfg, std::vector<Vector3>& new_v, std::vector<int>& new_i);
}