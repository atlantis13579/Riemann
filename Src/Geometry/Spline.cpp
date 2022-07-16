
#include "Spline.h"

#include "math.h"
#include <functional>
#include <vector>

#include "../Solver/NewtonRaphsonIteration.h"

static float Combination(int n, int m)
{
	static std::vector<std::vector<float>> cache;
	if (n < cache.size() && m < cache[n].size())
	{
		return cache[n][m];
	}
	int ret = 1;
	for (int i = 0; i < m; i++) {
		ret = ret * (n - i) / (i + 1);
	}
	return float(ret);
}

Vector3d Bezier::Calculate(const std::vector<Vector3d> &control_points, float time)
{
	Vector3d pos(0, 0, 0);
	int n = (int)control_points.size() - 1;
	for (int i = 0; i <= n; ++i)
	{
		Vector3d p = control_points[i];
		pos = pos + p * (powf(1.0f - time, (float)(n - i)) * powf(time, (float)i) * Combination(n, i));
	}
	return pos;
}

Vector3d Bezier::Calculate(const Vector3d &p0, const Vector3d &p1, const Vector3d &p2, float time)
{
	Vector3d pos = p0 * ((1.0f - time) * (1.0f - time)) + p1 * ((1.0f - time) * time * 2.0f) + p2 * (time * time);
	return pos;
}

Vector3d Bezier::CalculateTangent(const std::vector<Vector3d> &control_points, float time)
{
	Vector3d pos(0, 0, 0);
	int n = (int)control_points.size() - 1;
	for (int i = 0; i <= n; ++i)
	{
		Vector3d p = control_points[i];
		pos = pos + p * (-1 * (n - i) * powf(1 - time, (float)(n - i - 1)) * powf(time, (float)i) * Combination(n, i))
				  + p * (i * powf(1 - time, (float)(n - i)) * powf(time, (float)(i - 1)) * Combination(n, i));
	}
	return pos;
}

Vector3d Bezier::CalculateTangent(const Vector3d &p0, const Vector3d &p1, const Vector3d &p2, float time)
{
	Vector3d pos = p0 * (2.0f * time - 2.0f) + p1 * (2.0f - 4.0f * time) + p2 * (2.0f * time);
	return pos;
}

float CalculateSegmentLength(const std::vector<Vector3d> &points)
{
	float sum = 0.0f;
	for (size_t i = 0; i < points.size() - 1; ++i)
	{
		float len = (points[i + 1] - points[i]).Length();
		sum += len;
	}
	return sum;
}

std::vector<Vector3d>  Bezier::Interpolation(const std::vector<Vector3d> &points, const Vector3d &end, float delta)
{
	const Vector3d &p0 = points.front();
	const Vector3d &p1 = points.back();
	Vector3d p2 = p1 + (end - p1) * delta;

	Vector3d cross = (p0 - p1).Cross(p2 - p1);
	if (cross.SquareLength() < 1.0f)
		return points;

	// float oldLen = ::CalculateSegmentLength(points);
	// float curveLen = Bezier::CalculateLength(p0, p1, p2);
	// if (curveLen < oldLen)
	//	return points;

	std::vector<Vector3d> interp;
	interp.push_back(points[0]);

	float time = 0.0f;
	for (size_t i = 0; i < points.size() - 1; ++i)
	{
		float len = (points[i + 1] - points[i]).Length();

		auto f = [&](float t) {
			Vector3d pos = Bezier::Calculate(p0, p1, p2, t);
			float dist = (pos - interp[i]).Length();
			return dist - len;
		};

		// auto df = [&](float t) {
		//	Vector3d pos = Bezier::Calculate(p0, p1, p2, t);
		//	Vector3d dev = Bezier::CalculateTangent(p0, p1, p2, t);
		//	return 2.0f * pos.Dot(dev);
		// };

		float guess = time + 1.0f / points.size();
		time = NewtonRaphson::FindRoot(f, guess);
		time = std::max(0.0f, std::min(1.0f, time));

		Vector3d pos = Bezier::Calculate(p0, p1, p2, time);
		interp.push_back(pos);
	}

	// float newLen = ::CalculateSegmentLength(interp);

	return interp;
}

float Bezier::CalculateLength(const Vector3d &p0, const Vector3d &p1, const Vector3d &p2)
{
	Vector3d a, b;
	a.x = p0.x - 2 * p1.x + p2.x;
	a.y = p0.y - 2 * p1.y + p2.y;
	b.x = 2 * p1.x - 2 * p0.x;
	b.y = 2 * p1.y - 2 * p0.y;
	float A = 4 * (a.x*a.x + a.y*a.y);
	float B = 4 * (a.x*b.x + a.y*b.y);
	float C = b.x*b.x + b.y*b.y;

	float Sabc = 2 * sqrtf(A + B + C);
	float A_2 = sqrtf(A);
	float A_32 = 2 * A*A_2;
	float C_2 = 2 * sqrtf(C);
	float BA = B / A_2;

	return (A_32*Sabc +
		A_2 * B*(Sabc - C_2) +
		(4 * C*A - B * B)*logf((2 * A_2 + BA + Sabc) / (BA + C_2))
		) / (4 * A_32);
};

// https://en.wikipedia.org/wiki/Cubic_Hermite_spline
Vector3d CubicHermite::Calculate(const Vector3d& p0, const Vector3d& p1, const Vector3d& t0, const Vector3d& t1, float t)
{
	float A = (2.0f * t * t * t - 3.0f * t * t + 1.0f);
	float B = (t * t * t - 2.0f * t * t + t);
	float C = (-2.0f * t * t * t + 3.0f * t * t);
	float D = (t * t * t - t * t);
	Vector3d Pos = A * p0 + B * t0 + C * p1 + D * t1;
	return Pos;
}

// https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull%E2%80%93Rom_spline
Vector3d CatmullRom::Calculate(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2, const Vector3d& p3, float t)
{
	t = std::max(0.0f, std::min(t, 1.0f));
	Vector3d a = 2.0f * p1;
	Vector3d b = p2 - p0;
	Vector3d c = 2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3;
	Vector3d d = -p0 + 3.0f * p1 - 3.0f * p2 + p3;
	Vector3d Pos = 0.5f * (a + (b * t) + (c * t * t) + (d * t * t * t));
	return Pos;
}

// https://en.wikipedia.org/wiki/Curvature#General_expressions
float CatmullRom::Curvature(const Vector3d& a, const Vector3d& b, const Vector3d& c, const Vector3d& d, float t)
{
	float tt = t * t;
	// float dx = 3.0f * a.x * tt + 2.0f * b.x * t + c.x;
	// float ddx = 6.0f * a.x * t + 2.0f * b.x;
	// float dy = 3.0f * a.y * tt + 2.0f * b.y * t + c.y;
	// float ddy = 6.0f * a.y * t + 2.0f * b.y;
	// float dz = 3.0f * a.z * tt + 2.0f * b.z * t + c.z;
	// float ddz = 6.0f * a.z * t + 2.0f * b.z;
	// float curvature = sqrtf((ddz * dy - ddy * dz) * (ddz * dy - ddy * dz) +
	//						(ddx * dz - ddz * dx) * (ddx * dz - ddz * dx) +
	//						(ddy * dx - ddx * dy) * (ddy * dx - ddx * dy)) / powf(dx * dx + dy * dy + dz * dz, 1.5f);
	Vector3d DL = 3.0f * a * tt + 2.0f * b * t + c;
	Vector3d DDL = 6.0f * a * t + 2.0f * b;
	float curvature = (DL.Cross(DDL)).Length() / std::max(DL.Length(), 1e-6f);
	curvature = curvature < 1e-6f ? 0.0f : curvature;
	return curvature;
}

// https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html
std::vector<SplineNode> CatmullRom::Smoothing(const std::vector<Vector3d>& points, float dlen, float alpha, float tension)
{
	std::vector<SplineNode> smoothed;
	if (points.size() == 0)
	{
		return smoothed;
	}
	else if (points.size() == 1)
	{
		smoothed.emplace_back(points[0], 0.0f);
		return smoothed;
	}

	for (size_t i = 0; i < points.size() - 1; ++i)
	{
		Vector3d p0 = i == 0 ? 2.0f * points[0] - points[1] : points[i - 1];
		Vector3d p1 = points[i];
		Vector3d p2 = points[i+1];
		Vector3d p3 = i == points.size() - 2 ? 2.0f * points[points.size() - 1] - points[points.size() - 2] : points[i + 2];

		float t01 = powf((p0 - p1).Length(), alpha);
		float t12 = powf((p1 - p2).Length(), alpha);
		float t23 = powf((p2 - p3).Length(), alpha);

		Vector3d m1 = (1.0f - tension) *
			(p2 - p1 + t12 * ((p1 - p0) / t01 - (p2 - p0) / (t01 + t12)));
		Vector3d m2 = (1.0f - tension) *
			(p2 - p1 + t12 * ((p3 - p2) / t23 - (p3 - p1) / (t12 + t23)));

		Vector3d a = 2.0f * (p1 - p2) + m1 + m2;
		Vector3d b = -3.0f * (p1 - p2) - m1 - m1 - m2;
		Vector3d c = m1;
		Vector3d d = p1;

		int n = (int)((p1 - p2).Length() / dlen + 0.5f);
		for (int i = 0; i < n; ++i)
		{
			float t = i * 1.0f / n;
			Vector3d point = a * t * t * t + b * t * t + c * t + d;
			float curvature = Curvature(a, b, c, d, t);
			smoothed.emplace_back(point, curvature);
		}
	}

	smoothed.emplace_back(points.back(), 0.0f);
	// return std::move(smoothed);
	return smoothed;
}
