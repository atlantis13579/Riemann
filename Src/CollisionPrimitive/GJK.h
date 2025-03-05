#pragma once

#include <float.h>
#include "MinkowskiSum.h"
#include "Simplex.h"

// Gilbert–Johnson–Keerthi algorithm
// http://allenchou.net/2013/12/game-physics-collision-detection-gjk/
// https://en.wikipedia.org/wiki/Gilbert%E2%80%93Johnson%E2%80%93Keerthi_distance_algorithm

namespace Riemann
{
	enum class GJK_status
	{
		NotIntersect,
		Intersect,
		Failed
	};

	#define GJK_MAX_ITERATIONS		(128)
	#define GJK_ACCURACY			(0.0001f)
	#define GJK_MIN_DISTANCE		(0.0001f)
	#define GJK_DUPLICATED_EPS		(0.0001f)

	class GJKIntersection
	{
	public:
		Simplex			result;
		float			distance;

		GJK_status Solve(MinkowskiSum* shape, int maxIterations = GJK_MAX_ITERATIONS)
		{
			Vector3 InitGuess = shape->Center();

			distance = -1.0f;
			result.Init(shape);
			result.AddPoint(InitGuess.SquareLength() > 0 ? InitGuess : -Vector3::UnitX());
			Vector3 dir = result.v[0].pos;

			int nlastp = 0;
			Vector3 lastp[4] = { dir, dir, dir, dir };

			float max_acc = 0.0f;
			int iter = 0;
			while (iter++ < maxIterations)
			{
				distance = dir.Length();
				if (distance < GJK_MIN_DISTANCE)
				{
					return GJK_status::Intersect;
				}

				result.AddPoint(-dir);

				const Vector3& p = result.LastPoint();
				if (IsDuplicated(lastp, p))
				{
					// Find duplicate point, stop searching to avoid infinite loop
					result.RemovePoint();
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
					result.RemovePoint();
					break;
				}

				int mask = 0;
				if (result.ProjectOrigin(dir, mask))
				{
					result.Update(mask);

					if (mask == 0b1111)
					{
						return GJK_status::Intersect;
					}
				}
				else
				{
					result.RemovePoint();
					break;
				}
			};

			return iter >= maxIterations ? GJK_status::Failed : GJK_status::NotIntersect;
		}

	private:
		static bool IsDuplicated(Vector3 lastp[4], const Vector3& p)
		{
			for (int i = 0; i < 4; ++i)
			{
				Vector3 diff = p - lastp[i];
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
			GJK_status status = gjk.Solve(Shape);

			if (status == GJK_status::NotIntersect)
			{
				return gjk.distance;
			}
			if (status == GJK_status::Intersect)
			{
				return -1.0f;
			}
			return FLT_MAX;
		}
	};
}