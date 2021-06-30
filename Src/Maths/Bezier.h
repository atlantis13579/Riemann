
#pragma once

#include <vector>
#include "Vector3d.h"

class Bezier
{
public:
	static Vector3d Calculate(const std::vector<Vector3d> &control_points, float time);
	static Vector3d Calculate(const Vector3d &p0, const Vector3d &p1, const Vector3d &p2, float time);
	static Vector3d CalculateTangent(const std::vector<Vector3d> &control_points, float time);
	static Vector3d CalculateTangent(const Vector3d &p0, const Vector3d &p1, const Vector3d &p2, float time);
	static float    CalculateLength(const Vector3d &p0, const Vector3d &p1, const Vector3d &p2);

	static std::vector<Vector3d>    Interpolation(const std::vector<Vector3d> &points, const Vector3d &end, float delta);
};

