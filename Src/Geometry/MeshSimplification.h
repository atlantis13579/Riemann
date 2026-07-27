#pragma once

#include <vector>
#include "../Maths/Vector3.h"

namespace Riemann
{
	struct SimplificationConfig
	{
		float	rate = 1.0f;
		int		faces = -1;
	};

	// faster
	bool SimplifyMeshGarlandQEM(const Vector3* pv, const void* pi, int nv, int nt, bool is16bit, const SimplificationConfig& cfg, std::vector<Vector3>& new_v, std::vector<int>& new_i);

	// better
	bool SimplifyMeshHoppeQEM(const Vector3* pv, const void* pi, int nv, int nt, bool is16bit, const SimplificationConfig& cfg, std::vector<Vector3>& new_v, std::vector<int>& new_i);

	// default router
	inline bool SimplifyMesh(const Vector3* pv, const void* pi, int nv, int nt, bool is16bit, const SimplificationConfig& cfg, std::vector<Vector3>& new_v, std::vector<int>& new_i)
	{
		return SimplifyMeshGarlandQEM(pv, pi, nv, nt, is16bit, cfg, new_v, new_i);
	}
}
