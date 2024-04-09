#pragma once

#include <stdint.h>
#include <vector>

#include "../Maths/Vector2.h"
#include "Segment2d.h"
#include "ShapeType.h"

namespace Riemann
{
	class Triangle2d
	{
	public:
		Vector2 v0, v1, v2;

	public:
		Triangle2d()
		{
		}

		Triangle2d(const Vector2& InA, const Vector2& InB, const Vector2& InC)
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
	};

	struct Segment2dTriangle2dIntersectionResult
	{
		int Quantity = 0;
		IntersectionType Type = IntersectionType::Empty;

		Vector2 Point0;
		Vector2 Point1;
	};

	bool CalculateIntersectionSegment2dTriangle2d(const Segment2d& segment, const Triangle2d& triangle, Segment2dTriangle2dIntersectionResult& Result);
}