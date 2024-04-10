#pragma once

#include <vector>
#include "../Maths/Vector2.h"
#include "../Maths/Vector3.h"
#include "../Maths/Box2.h"

namespace Geometry
{
	class Voronoi2
	{
	public:
		static void GenerateRandomPoints(const Box2& Bounds, int numPoints, std::vector<Vector2>& points);
	};
}