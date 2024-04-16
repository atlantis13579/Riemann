#pragma once

#include "../Maths/Box1.h"
#include "../Maths/Box2.h"
#include "../Maths/Vector2.h"

namespace Riemann
{
	class Triangle2
	{
	public:
		Vector2 v0, v1, v2;

	public:
		Triangle2()
		{
		}

		Triangle2(const Vector2& InA, const Vector2& InB, const Vector2& InC)
		{
			Init(InA, InB, InC);
		}

		void	Init(const Vector2& InA, const Vector2& InB, const Vector2& InC)
		{
			v0 = InA;
			v1 = InB;
			v2 = InC;
		}

		Vector2& operator[](int i)
		{
			return (&v0)[i];
		}

		const Vector2& operator[](int i) const
		{
			return (&v0)[i];
		}		

		const Vector2* GetVertex() const
		{
			return &v0;
		}

		Vector2*	GetVertex()
		{
			return &v0;
		}

		Vector2		GetCenter() const
		{
			return (v0 + v1 + v2) / 3.0f;
		}

		static float Area2D(float x1, float y1, float x2, float y2, float x3, float y3)
		{
			return (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2);
		}

		float		GetArea() const
		{
			return Area2D(v0.x, v0.y, v1.x, v1.y, v2.x, v2.y);
		}

		Box2		GetBounds() const
		{
			Box2 box(v0, v1);
			box.Encapsulate(v2);
			return box;
		}

		int IsInside(const Vector2& p) const
		{
			int Sign1 = Orient(v0, v1, p);
			if (Sign1 > 0)
			{
				return 1;
			}

			int Sign2 = Orient(v1, v2, p);
			if (Sign2 > 0)
			{
				return 1;
			}

			int Sign3 = Orient(v0, v2, p);
			if (Sign3 < 0)
			{
				return 1;
			}

			return (Sign1 != 0 && Sign2 != 0 && Sign3 != 0) ? -1 : 0;
		}

		struct Segment2IntersectionResult
		{
			int Quantity = 0;
			IntersectionType Type = IntersectionType::Empty;

			Vector2 Point0;
			Vector2 Point1;
		};

		Vector3 BarycentricCoods(const Vector2& Point) const
		{
			return BarycentricCoods(Point, v0, v1, v2);
		}

		static Vector3 BarycentricCoods(const Vector2& Point, const Vector2& v0, const Vector2& v1, const Vector2& v2)
		{
			Vector2 v02 = v0 - v2;
			Vector2 v12 = v1 - v2;
			Vector2 vp2 = Point - v2;
			float d00 = v02.Dot(v02);
			float d01 = v02.Dot(v12);
			float d11 = v12.Dot(v12);
			float d20 = v02.Dot(vp2);
			float d21 = v12.Dot(vp2);
			float denom = d00 * d11 - d01 * d01;
			float u = (d11 * d20 - d01 * d21) / denom;
			float v = (d00 * d21 - d01 * d20) / denom;
			float w = 1.0f - u - v;
			return Vector3(u, v, w);
		}

		bool IntersectSegment(const Vector2& P0, const Vector2 &P1) const
		{
			// TODO
			return false;
		}

		bool IntersectSegment(const Vector2& P0, const Vector2& P1, Segment2IntersectionResult& Result) const
		{
			const Triangle2& triangle = *this;

			const float Tolerance = 1e-6f;

			Vector2 Center = (P1 + P0) * 0.5f;
			float Extent = (P1 - P0).Length() * 0.5f;

			if (Extent < 1e-6f)				// segment is a point
			{
				int pos = 0, neg = 0;
				for (int i = 2, j = 0; j < 3; i = j++)
				{
					Vector2 p = Center - triangle[j];
					Vector2 perp = (triangle[j] - triangle[i]).PerpCW();
					float len = perp.SafeNormalize();
					if (len == 0)
					{
						Vector2 other = triangle[(j + 1) % 3];
						perp = other - triangle[j];
						len = perp.SafeNormalize();
						if (len == 0)
						{
							if ((triangle[0] - Center).SquareLength() <= Tolerance * Tolerance)
							{
								pos = neg = 0;
							}
							else
							{
								pos = neg = 1;
							}
							break;
						}
						else
						{
							Vector2 p_other = Center - other;
							Vector2 minus_perp = -perp;
							float dp = minus_perp.Dot(p_other);
							if (dp < -Tolerance)
							{
								neg++;
							}
							else if (dp > Tolerance)
							{
								pos++;
							}
						}
					}

					float dp = perp.Dot(p);
					if (dp < -Tolerance)
					{
						neg++;
					}
					else if (dp > Tolerance)
					{
						pos++;
					}
				}

				if (pos == 0 || neg == 0)
				{
					Result.Type = IntersectionType::Segment;
					Result.Quantity = 2;
					Result.Point0 = Center;
					Result.Point1 = Center;
					return true;
				}
				else
				{
					Result.Type = IntersectionType::Empty;
					return false;
				}
			}

			Vector2 Direction = (P1 - P0).Unit();

			float dist[3];
			int sign[3];
			int positive = 0, negative = 0, zero = 0;
			TriangleLineRelations(Center, Direction, triangle, dist, sign, positive, negative, zero, Tolerance);

			if (positive == 3 || negative == 3)
			{
				// No intersections.
				Result.Quantity = 0;
				Result.Type = IntersectionType::Empty;
			}
			else
			{
				Vector2 param;
				GetInterval(Center, Direction, triangle, dist, sign, param);

				Box1 segment1(param[0], param[1]);
				Box1 segment2(-Extent, Extent);
				Box1 intr;

				if (segment1.GetIntersection(segment2, intr))
				{
					if (!intr.IsPoint())
					{
						// Segment intersection.
						Result.Type = IntersectionType::Segment;
						Result.Point0 = Center + intr.Min * Direction;
						Result.Point1 = Center + intr.Max * Direction;
					}
					else if (Result.Quantity == 1)
					{
						// Point intersection.
						Result.Type = IntersectionType::Point;
						Result.Point0 = Center + intr.Min * Direction;
					}
				}
				else
				{
					// No intersections.
					Result.Type = IntersectionType::Empty;
				}

			}

			return Result.Type != IntersectionType::Empty;
		}

	private:
		static void TriangleLineRelations(
			const Vector2& Origin, const Vector2& Direction, const Triangle2& Tri,
			float Dist[3], int Sign[3], int& Positive, int& Negative, int& Zero, const float Tolerance = 1e-6f)
		{
			Positive = 0;
			Negative = 0;
			Zero = 0;
			for (int i = 0; i < 3; ++i)
			{
				Vector2 diff = Tri[i] - Origin;
				Dist[i] = DotPerp(diff, Direction);
				if (Dist[i] > Tolerance)
				{
					Sign[i] = 1;
					++Positive;
				}
				else if (Dist[i] < -Tolerance)
				{
					Sign[i] = -1;
					++Negative;
				}
				else
				{
					Dist[i] = 0.0f;
					Sign[i] = 0;
					++Zero;
				}
			}
		}

		static bool GetInterval(const Vector2& Origin, const Vector2& Direction, const Triangle2& Tri,
			const float Dist[3], const int Sign[3], Vector2& param)
		{
			float proj[3];
			int i;
			for (i = 0; i < 3; ++i)
			{
				Vector2 diff = Tri[i] - Origin;
				proj[i] = Direction.Dot(diff);
			}

			float numer, denom;
			int i0, i1;
			int quantity = 0;
			for (i0 = 2, i1 = 0; i1 < 3; i0 = i1++)
			{
				if (Sign[i0] * Sign[i1] < 0)
				{
					numer = Dist[i0] * proj[i1] - Dist[i1] * proj[i0];
					denom = Dist[i0] - Dist[i1];
					param[quantity++] = numer / denom;
				}
			}

			if (quantity < 2)
			{
				for (i = 0; i < 3; i++)
				{
					if (Sign[i] == 0)
					{
						if (quantity == 2)
						{
							if (param[0] > param[1])
							{
								std::swap(param[0], param[1]);
							}
							float extraparam = proj[i];
							if (extraparam < param[0])
							{
								param[0] = extraparam;
							}
							else if (extraparam > param[1])
							{
								param[1] = extraparam;
							}
						}
						else
						{
							param[quantity++] = proj[i];
						}
					}
				}
			}

			if (quantity <= 0)
			{
				return false;
			}

			if (quantity == 2)
			{
				if (param[0] > param[1])
				{
					float save = param[0];
					param[0] = param[1];
					param[1] = save;
				}
			}
			else
			{
				param[1] = param[0];
			}

			return true;
		}
	};
}