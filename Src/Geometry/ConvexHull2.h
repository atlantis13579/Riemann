
#pragma once

#include <vector>
#include "../Maths/Vector2.h"

namespace Geometry
{
	// Find the ConvexHull, O(N*logN) time, O(N) space
	void ConvexHull_GrahamScan(const std::vector<Vector2>& points, std::vector<Vector2>* hull);

	// Find the ConvexHull, O(N*M) time, O(1) space
	void ConvexHull_GiftWrapping(const std::vector<Vector2>& points, std::vector<Vector2>* hull);

	// Given a convex polygon, find the i1, i2 which maximize the |convex[i1] - convex[i2]|
	void RotatingCaliper(const std::vector<Vector2>& convex, int* i1, int* i2);
}