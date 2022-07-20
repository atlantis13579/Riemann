#pragma once

#include <float.h>
#include "MinkowskiSum.h"
#include "Simplex.h"

// Gilbert–Johnson–Keerthi algorithm
// http://allenchou.net/2013/12/game-physics-collision-detection-gjk/
// https://en.wikipedia.org/wiki/Gilbert%E2%80%93Johnson%E2%80%93Keerthi_distance_algorithm

enum class GJK_result
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
	float			distance;

	GJK_result Solve(MinkowskiSum* shape)
	{
		Vector3d InitGuess = shape->Center();

		distance = -1.0f;
		simplex.Init(shape);
		simplex.AddPoint(InitGuess.SquareLength() > 0 ? InitGuess : -Vector3d::UnitX());
		Vector3d dir = simplex.v[0].pos;

		int nlastp = 0;
		Vector3d lastp[4] = { dir, dir, dir, dir };

		float max_acc = 0.0f;
		int iter = 0;
		while (iter++ < GJK_MAX_ITERATIONS)
		{
			distance = dir.Length();
			if (distance < GJK_MIN_DISTANCE)
			{
				return GJK_result::Inside;
			}

			simplex.AddPoint(-dir);

			const Vector3d& p = simplex.LastPoint();
			if (IsDuplicated(lastp, p))
			{
				// Find duplicate point, stop searching to avoid infinite loop
				simplex.RemovePoint();
				break;
			}
			else
			{
				nlastp = (nlastp + 1) & 3;
				lastp[nlastp] = p;
			}

			float acc = DotProduct(dir, p) / distance;
			max_acc = acc > max_acc ? acc : max_acc;

			// Exceed accuracy, stop searching
			if (distance - max_acc <= GJK_ACCURACY * distance)
			{
				simplex.RemovePoint();
				break;
			}

			int mask = 0;
			if (simplex.ProjectOrigin(dir, mask))
			{
				simplex.UpdatePointSet(mask);

				if (mask == 0b1111)
				{
					return GJK_result::Inside;
				}
			}
			else
			{
				simplex.RemovePoint();
				break;
			}
		};

		return iter >= GJK_MAX_ITERATIONS ? GJK_result::Failed : GJK_result::Separate;
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

class GJKClosestDistance
{
public:
	float Solve(MinkowskiSum* Shape)
	{
		GJKIntersection gjk;
		GJK_result result = gjk.Solve(Shape);

		if (result == GJK_result::Separate)
		{
			return gjk.distance;
		}
		if (result == GJK_result::Inside)
		{
			return -1.0f;
		}
		else if (result == GJK_result::Boundary)
		{
			return 0.0f;
		}
		return FLT_MAX;
	}
};
