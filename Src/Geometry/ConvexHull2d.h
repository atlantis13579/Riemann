
#pragma once

#include <vector>
#include "../Maths/Vector2.h"

// Find the ConvexHull, O(N*logN) time, O(N) space
void ConvexHull_GrahamScan(const std::vector<Vector2d>& points, std::vector<Vector2d>* hull);

// Find the ConvexHull, O(N*M) time, O(1) space
void ConvexHull_GiftWrapping(const std::vector<Vector2d> &points, std::vector<Vector2d>* hull);

// Given a convex polygon, find the i1, i2 which maximize the |convex[i1] - convex[i2]|
void RotatingCaliper(const std::vector<Vector2d>& convex, int* i1, int* i2);
