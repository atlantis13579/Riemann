#pragma once

#include "../Maths/Box1.h"
#include "../Maths/Vector2.h"
#include "Segment2.h"

namespace Geometry
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

		static float Area2D(float x1, float y1, float x2, float y2, float x3, float y3)
		{
			return (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2);
		}

		float CalculateArea() const
		{
			return Area2D(v0.x, v0.y, v1.x, v1.y, v2.x, v2.y);
		}

		int IsInside(const Vector2& p) const
		{
			float Sign1 = Orient(v0, v1, p);
			if (Sign1 > 0)
			{
				return 1;
			}

			float Sign2 = Orient(v1, v2, p);
			if (Sign2 > 0)
			{
				return 1;
			}

			float Sign3 = Orient(v0, v2, p);
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

		bool CalculateIntersectionSegment2(const Segment2& segment, Segment2IntersectionResult& Result) const
		{
			const Triangle2& triangle = *this;

			const float Tolerance = 1e-6f;
			Vector2 Center = segment.GetCenter();
			float Extent = segment.GetExtent();

			if (segment.GetLength() < 1e-6f)				// segment is a point
			{
				int pos = 0, neg = 0;
				for (int TriPrev = 2, TriIdx = 0; TriIdx < 3; TriPrev = TriIdx++)
				{
					Vector2 ToPt = Center - triangle[TriIdx];
					Vector2 EdgePerp = (triangle[TriIdx] - triangle[TriPrev]).PerpCW();
					float EdgeLen = EdgePerp.SafeNormalize();
					if (EdgeLen == 0)
					{
						Vector2 OtherV = triangle[(TriIdx + 1) % 3];
						EdgePerp = OtherV - triangle[TriIdx];
						EdgeLen = EdgePerp.SafeNormalize();
						if (EdgeLen == 0)
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
							Vector2 ToPtFromOther = Center - OtherV;
							Vector2 BackwardsEdgePerp = -EdgePerp;
							float OtherSideSign = BackwardsEdgePerp.Dot(ToPtFromOther);
							if (OtherSideSign < -Tolerance)
							{
								neg++;
							}
							else if (OtherSideSign > Tolerance)
							{
								pos++;
							}
						}
					}

					float SideSign = EdgePerp.Dot(ToPt);
					if (SideSign < -Tolerance)
					{
						neg++;
					}
					else if (SideSign > Tolerance)
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

			Vector2 Direction = segment.GetDirection();

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