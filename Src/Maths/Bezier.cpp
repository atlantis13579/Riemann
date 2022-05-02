
#include "Bezier.h"

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
