#pragma once

#include <vector>
#include "../Maths/Vector3.h"

namespace Riemann
{
	struct SplineNode
	{
		SplineNode() {}
		SplineNode(const SplineNode& _n)
		{
			point = _n.point;
			curvature = _n.curvature;
		}
		SplineNode(const Vector3& _p, float _c)
		{
			point = _p;
			curvature = _c;
		}
		Vector3	point;
		float		curvature;
	};

	class Bezier
	{
	public:
		static Vector3 Calculate(const std::vector<Vector3>& control_points, float time);
		static Vector3 Calculate(const Vector3& p0, const Vector3& p1, const Vector3& p2, float time);
		static Vector3 CalculateTangent(const std::vector<Vector3>& control_points, float time);
		static Vector3 CalculateTangent(const Vector3& p0, const Vector3& p1, const Vector3& p2, float time);
		static float    CalculateLength(const Vector3& p0, const Vector3& p1, const Vector3& p2);

		static std::vector<Vector3>    Interpolation(const std::vector<Vector3>& points, const Vector3& end, float delta);
	};

	class CubicHermite
	{
	public:
		static Vector3	Calculate(const Vector3& p0, const Vector3& p1, const Vector3& t0, const Vector3& t1, float t);
	};

	class CatmullRom
	{
	public:
		static Vector3	Calculate(const Vector3& p0, const Vector3& p1, const Vector3& t0, const Vector3& t1, float t);
		static float	Curvature(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d, float t);

		// alpha = 0.0 : Uniform Catmull-Rom curve 
		// alpha = 0.5 : Centripetal Catmull-Rom curve
		// alpha = 1.0 : Chordal Catmull-Rom curves
		// A good value for alpha is 0.5 which gives us a centripetal Catmull-Rom spline, and for tension a value 0 is a good choice
		static std::vector<SplineNode>    Smoothing(const std::vector<Vector3>& points, float dt, float alpha = 0.5f, float tension = 0.0f);
	};
}