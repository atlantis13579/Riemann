#pragma once

#include "../Maths/Vector2.h"

namespace Riemann
{
	class Segment2d
	{
	public:
		Vector2	P0;
		Vector2	P1;

		Segment2d(const Vector2& _p0, const Vector2& _p1)
		{
			P0 = _p0;
			P1 = _p1;
		}

		Vector2		GetDirection() const
		{
			return (P1 - P0).SafeUnit();
		}

		float		GetLength() const
		{
			return (P1 - P0).Length();
		}

		float		GetExtent() const
		{
			return (P1 - P0).Length() * 0.5f;
		}

		Vector2		GetCenter() const
		{
			return (P1 + P0) * 0.5f;
		}
	};
}