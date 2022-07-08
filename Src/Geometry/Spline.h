
#pragma once

#include <vector>
#include "../Maths/Vector3d.h"

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

class CubicHermite
{
public:
	static Vector3d	Calculate(const Vector3d& p0, const Vector3d& p1, const Vector3d& t0, const Vector3d& t1, float t);
};

class CatmullRom
{
public:
	static Vector3d	Calculate(const Vector3d& p0, const Vector3d& p1, const Vector3d& t0, const Vector3d& t1, float t);

	// alpha = 0.0 : Uniform Catmull-Rom curve 
	// alpha = 0.5 : Centripetal Catmull-Rom curve
	// alpha = 1.0 : Chordal Catmull-Rom curves
	// A good value for alpha is 0.5 which gives us a centripetal Catmull-Rom spline, and for tension a value 0 is a good choice
	static std::vector<Vector3d>    Smoothing(const std::vector<Vector3d>& points, float dt, float alpha = 0.5f, float tension = 0.0f);
};