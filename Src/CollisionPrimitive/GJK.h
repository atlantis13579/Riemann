#pragma once

#include <float.h>
#include "MinkowskiSum.h"
#include "Simplex.h"

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

	// Gilbert–Johnson–Keerthi algorithm
	// http://allenchou.net/2013/12/game-physics-collision-detection-gjk/
	// https://en.wikipedia.org/wiki/Gilbert%E2%80%93Johnson%E2%80%93Keerthi_distance_algorithm
	class GJKIntersection
	{
	public:
		Simplex			result;
		float			distance;

		GJK_status Solve(const GjkShape* shape)
		{
			Vector3 InitGuess = shape->Center();

			distance = -1.0f;
			result.Init(shape);
			result.AddPoint(InitGuess.SquareLength() > 0 ? InitGuess : -Vector3::UnitX());
			Vector3 v = result.v[0].pos;

			int nlastp = 0;
			Vector3 lastp[4] = { v, v, v, v };

			const int maxIterations = GJK_MAX_ITERATIONS;
			float max_acc = 0.0f;
			int iter = 0;
			while (iter++ < maxIterations)
			{
				distance = v.Length();
				if (distance < GJK_MIN_DISTANCE)
				{
					return GJK_status::Intersect;
				}

				result.AddPoint(-v);

				const Vector3& p = result.LastPoint();
				if (CheckDuplicated(lastp, p))
				{
					// Find duplicate point, stop searching to avoid infinite loop
					break;
				}
				else
				{
					nlastp = (nlastp + 1) & 3;
					lastp[nlastp] = p;
				}

				float acc = DotProduct(v, p) / distance;
				max_acc = acc > max_acc ? acc : max_acc;

				// Exceed accuracy, stop searching
				if (distance - max_acc <= GJK_ACCURACY * distance)
				{
					break;
				}

				unsigned int mask = 0;
				float prev_sq_length = v.SquareLength();
				if (!result.GetClosestToOrigin(v, mask))
				{
					break;
				}

				float sq_length = v.SquareLength();
				if (sq_length >= prev_sq_length)
				{
					break;		// we've converged, stop searching
				}

				result.Update(mask);

				if (mask == 0b1111)
				{
					// inside the tetrahedron
					assert(sq_length < GJK_MIN_DISTANCE);
					return GJK_status::Intersect;
				}
			};

			return iter >= maxIterations ? GJK_status::Failed : GJK_status::NotIntersect;
		}

		template<class Shape1, class Shape2>
		GJK_status Solve(const Shape1* shape1, const Shape2* shape2)
		{
			MinkowskiSum<Shape1, Shape2> sum(shape1, shape2);
			GJK_status status = Solve(&sum);
			return status;
		}

	private:
		static bool CheckDuplicated(Vector3 lastp[4], const Vector3& p)
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
		float Solve(GjkShape* Shape)
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

	// Ray Casting against General Convex Objects with Application to Continuous Collision Detection
	// http://dtecta.com/papers/jgt04raycast.pdf
	class GJKRaycast
	{
	public:
		float	time;
		Vector3	normal;
		Vector3 position;

		GJK_status Solve(const Vector3& Origin, const Vector3& Direction, const GjkShape* shape)
		{
			Vector3		mY[4];
			Vector3		mP[4];
			int			dimension = 0;
			Simplex		result;
			const float max_t = FLT_MAX;

			float ht = 0.0f;
			Vector3 x = Origin;
			Vector3 v = x - shape->Center();
			bool allow_restart = false;

			const int maxIterations = GJK_MAX_ITERATIONS;
			int iter = 0;
			while (iter++ < maxIterations)
			{
				Vector3 p = shape->Support(v);
				Vector3 w = x - p;

				float vw = v.Dot(w);

				if (vw > 0.0f)
				{
					float vr = v.Dot(Direction);
					if (vr >= 0.0f)
					{
						break;
					}

					float delta = vw / vr;

					// Exceed accuracy, stop searching
					if (ht - delta <= GJK_ACCURACY * ht)
					{
						break;
					}
					ht -= delta;

					if (ht >= max_t)
					{
						break;
					}

					x = Origin + ht * Direction;

					// We allow rebuilding the simplex once after x changes because the simplex was built
					// for another x and numerical round off builds up as you keep adding points to an
					// existing simplex
					allow_restart = true;
				}

				mP[dimension++] = p;

				for (int i = 0; i < dimension; ++i)
					mY[i] = x - mP[i];

				result.SetPoints(dimension, mY);

				float prev_sq_length = v.SquareLength();
				unsigned int mask = 0;
				if (!result.GetClosestToOrigin(v, mask))
				{
					// Only allow 1 restart, if we still can't get a closest point
					// we're so close that we return this as a hit
					if (!allow_restart)
						break;

					allow_restart = false;
					mP[0] = p;
					dimension = 1;
					v = x - p;
					continue;
				}

				const float sq_length = v.SquareLength();
				if (sq_length >= prev_sq_length)
				{
					break;		// we've converged, stop searching
				}

				if (mask == 0b1111)
				{
					// inside the tetrahedron
					assert(sq_length < GJK_MIN_DISTANCE);
					time = ht;
					return GJK_status::Intersect;
				}

				result.Update(mask);

				if (sq_length <= GJK_MIN_DISTANCE * GJK_MIN_DISTANCE)
				{
					time = ht;
					return GJK_status::Intersect;
				}
			}

			return iter >= maxIterations ? GJK_status::Failed : GJK_status::NotIntersect;
		}
	};

	class GJKShapecast
	{
	public:
		template<class CastShape, class Shape>
		bool Solve(const Vector3& Direction, const CastShape* cast_shape, const Shape* shape, Vector3* p, Vector3* n, float* t)
		{
			MinkowskiSum<Shape, CastShape> sum(shape, cast_shape);
			GJKRaycast gjk;
			GJK_status status = gjk.Solve(Vector3::Zero(), Direction, &sum);
			if (status == GJK_status::Intersect)
			{
				*t = gjk.time;
				*n = gjk.normal;
				*p = gjk.position;
				return true;
			}
			return false;
		}
	};
}