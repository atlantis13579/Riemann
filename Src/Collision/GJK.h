#pragma once

#include "Minkowski.h"
#include "../Maths/Vector3d.h"
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
	Simplex			cs;

	GJK_status Solve(MinkowskiSum* Shape, Vector3d InitGuess)
	{
		cs.AddVertex(InitGuess.SquareLength() > 0 ? -InitGuess : Vector3d::UnitX(), 1, Shape);
		Vector3d Dir = cs.v[0].p;

		int clastw = 0;
		Vector3d lastw[4] = { Dir, Dir, Dir, Dir };

		float alpha = 0.0f;

		int iter = 0;
		while (iter ++ < GJK_MAX_ITERATIONS)
		{
			float rl = Dir.Length();
			if (rl < GJK_MIN_DISTANCE)
			{
				return GJK_status::Inside;
			}

			cs.AddVertex(-Dir, 0, Shape);
			const Vector3d& w = cs.v[cs.dimension - 1].p;
			bool found = false;
			for (int i = 0; i < 4; ++i)
			{
				Vector3d diff;
				diff = w - lastw[i];
				if (diff.SquareLength() < GJK_DUPLICATED_EPS)
				{
					found = true;
					break;
				}
			}
			if (found)
			{
				cs.RemoveVertex();
				break;
			}
			else
			{
				lastw[clastw = (clastw + 1) & 3] = w;
			}

			float omega = DotProduct(Dir, w) / rl;
			alpha = omega > alpha ? omega : alpha;
			if (((rl - alpha) - (GJK_ACCURACY * rl)) <= 0)
			{
				cs.RemoveVertex();
				break;
			}

			float weights[4];
			int mask = 0;
			float SqrDist = cs.ProjectOrigin(weights, mask);

			if (SqrDist >= 0)
			{
				Simplex ns;
				Dir = Vector3d::Zero();
				for (int i = 0, ni = cs.dimension; i < ni; ++i)
				{
					if (mask & (1 << i))
					{
						ns.v[ns.dimension] = cs.v[i];
						ns.w[ns.dimension++] = weights[i];
						Dir = Dir + cs.v[i].p * weights[i];
					}
				}
				cs = ns;

				if (mask == 15)
				{
					return GJK_status::Inside;
				}
			}
			else
			{
				cs.RemoveVertex();
				break;
			}
		};

		return iter >= GJK_MAX_ITERATIONS ? GJK_status::Failed : GJK_status::Valid;
	}
};

