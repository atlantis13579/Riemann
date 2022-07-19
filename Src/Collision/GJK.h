#pragma once

#include "MinkowskiSum.h"
#include "Simplex.h"

// Gilbert–Johnson–Keerthi algorithm
// http://allenchou.net/2013/12/game-physics-collision-detection-gjk/
// https://en.wikipedia.org/wiki/Gilbert%E2%80%93Johnson%E2%80%93Keerthi_distance_algorithm

enum class GJK_status
{
	Separate,
	Inside,
	Boundary,
	Failed
};

#define GJK_MAX_ITERATIONS		(128)
#define GJK_ACCURACY			(0.0001f)
#define GJK_MIN_DISTANCE		(0.0001f)
#define GJK_DUPLICATED_EPS		(0.0001f)

class GJKIntersection
{
public:
	Simplex			simplex;

	GJK_status Solve(MinkowskiSum* Shape, Vector3d InitGuess)
	{
		simplex.AddPoint(InitGuess.SquareLength() > 0 ? -InitGuess : Vector3d::UnitX(), 1.0f, Shape);
		Vector3d SearchDir = simplex.v[0].p;

		int nlastp = 0;
		Vector3d lastp[4] = { SearchDir, SearchDir, SearchDir, SearchDir };

		float max_omega = 0.0f;

		int iter = 0;
		while (iter++ < GJK_MAX_ITERATIONS)
		{
			float rl = SearchDir.Length();
			if (rl < GJK_MIN_DISTANCE)
			{
				return GJK_status::Inside;
			}

			simplex.AddPoint(-SearchDir, 0, Shape);

			const Vector3d& p = simplex.LastPoint();
			if (IsDuplicated(lastp, p))
			{
				simplex.RemovePoint();
				break;
			}
			else
			{
				lastp[nlastp = (nlastp + 1) & 3] = p;
			}

			float omega = DotProduct(SearchDir, p) / rl;
			max_omega = omega > max_omega ? omega : max_omega;
			if (((rl - max_omega) - (GJK_ACCURACY * rl)) <= 0)
			{
				simplex.RemovePoint();
				break;
			}

			int mask = 0;
			float SqrDist = simplex.ProjectOrigin(SearchDir, mask);
			if (SqrDist >= 0)
			{
				simplex.UpdatePointSet(mask);

				if (mask == 0b1111)
				{
					return GJK_status::Inside;
				}
			}
			else
			{
				simplex.RemovePoint();
				break;
			}
		};

		return iter >= GJK_MAX_ITERATIONS ? GJK_status::Failed : GJK_status::Separate;
	}

private:
	static bool IsDuplicated(Vector3d lastp[4], const Vector3d& p)
	{
		for (int i = 0; i < 4; ++i)
		{
			Vector3d diff = p - lastp[i];
			if (diff.SquareLength() < GJK_DUPLICATED_EPS)
			{
				return true;
			}
		}
		return false;
	}
};

class GJKClosest
{
public:
	Simplex			simplex;

	float Solve(MinkowskiSum* Shape, Vector3d InitGuess)
	{
		return -1.0f;
	}
};