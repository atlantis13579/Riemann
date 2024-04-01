#pragma once

#include <vector>
#include "../Maths/Vector3.h"
#include "../Maths/Box3d.h"

namespace Geometry
{
	void GenerateVoronoiSites(const Box3d& Bounds, int pointsMin, int pointsMax, std::vector<Vector3>& sites);
}