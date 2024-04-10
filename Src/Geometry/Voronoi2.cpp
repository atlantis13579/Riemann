
#include <assert.h>
#include <cmath>
#include "Voronoi2.h"

namespace Geometry
{
	void Voronoi2::GenerateRandomPoints(const Box2& Bounds, int numPoints, std::vector<Vector2>& points)
	{
		const Vector2 Extent(Bounds.Max - Bounds.Min);

		points.resize(numPoints);
		for (int i = 0; i < numPoints; ++i)
		{
			points[i] = Bounds.Min + Vector2::Random() * Extent;
		}
	}

}