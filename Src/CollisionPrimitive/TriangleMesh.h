
#pragma once

#include <vector>
#include "../Maths/Vector3d.h"

class TriangleMesh
{
public:
	std::vector<Vector3d>	Verties;
	std::vector<Vector3d>	Indices;

	int GetNumTriangles() const
	{
		return (int)Indices.size() / 3;
	}
};