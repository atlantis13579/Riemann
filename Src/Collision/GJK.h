#pragma once

#include "MinkowskiSum.h"
#include "Simplex.h"

// Gilbert–Johnson–Keerthi algorithm
// http://allenchou.net/2013/12/game-physics-collision-detection-gjk/

enum class  GJK_status
{
	Valid,
	Inside,
	Failed
};

#define GJK_MAX_ITERATIONS		(128)
#define GJK_ACCURACY			(0.0001f)
#define GJK_MIN_DISTANCE		(0.0001f)
#define GJK_DUPLICATED_EPS		(0.0001f)

class GJK
{
public:
	Simplex			simplex;

	GJK_status Solve(MinkowskiSum* Shape, Vector3d InitGuess)
	{
		simplex.AddPoint(InitGuess.SquareLength() > 0 ? -InitGuess : Vector3d::UnitX(), 1.0f, Shape);
		Vector3d Dir = simplex.v[0].p;

		int nlastp = 0;
		Vector3d lastp[4] = { Dir, Dir, Dir, Dir };

		float alpha = 0.0f;

		int iter = 0;
		while (iter++ < GJK_MAX_ITERATIONS)
		{
			float rl = Dir.Length();
			if (rl < GJK_MIN_DISTANCE)
			{
				return GJK_status::Inside;
			}

			simplex.AddPoint(-Dir, 0, Shape);

			const Vector3d& p = simplex.v[simplex.dimension - 1].p;
			if (IsDuplicated(lastp, p))
			{
				simplex.RemovePoint();
				break;
			}
			else
			{
				lastp[nlastp = (nlastp + 1) & 3] = p;
			}

			float omega = DotProduct(Dir, p) / rl;
			alpha = omega > alpha ? omega : alpha;
			if (((rl - alpha) - (GJK_ACCURACY * rl)) <= 0)
			{
				simplex.RemovePoint();
				break;
			}

			float coords[4];
			int mask = 0;
			float SqrDist = simplex.ProjectOrigin(coords, mask);

			if (SqrDist >= 0)
			{
				Dir = simplex.UpdatePointSet(coords, mask);

				if (mask == 0xf)
				{
					return GJK_status::Inside;
				}

				if (SqrDist < GJK_MIN_DISTANCE)
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

		return iter >= GJK_MAX_ITERATIONS ? GJK_status::Failed : GJK_status::Valid;
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
