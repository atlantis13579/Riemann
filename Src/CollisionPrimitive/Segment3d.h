#pragma once

#include "../Maths/Vector3.h"

class Segment3d
{
public:
	Vector3	P0;
	Vector3	P1;

	Segment3d(const Vector3& _p0, const Vector3& _p1)
	{
		P0 = _p0;
		P1 = _p1;
	}

	Vector3		GetDir() const
	{
		return (P1 - P0).SafeUnit();
	}

	float			SqrDistanceToPoint(const Vector3& Point) const
	{
		Vector3 Closest = Segment3d::ClosestPointOnSegment(Point, P0, P1);
		return (Point - Closest).SquareLength();
	}

	Vector3			ClosestPointTo(const Vector3& Point) const
	{
		return Segment3d::ClosestPointOnSegment(P0, P1, Point);
	}

	static float	SqrDistancePointToSegment(const Vector3& Point, const Vector3& P0, const Vector3& P1)
	{
		Vector3 Closest = Segment3d::ClosestPointOnSegment(Point, P0, P1);
		return (Point - Closest).SquareLength();
	}

	static Vector3 ClosestPointOnSegment(const Vector3& Point, const Vector3& P0, const Vector3& P1)
	{
		const Vector3 V1 = P1 - P0;
		const Vector3 V2 = Point - P0;

		const float dp1 = V2.Dot(V1);
		if (dp1 <= 0)
		{
			return P0;
		}

		const float dp2 = V1.Dot(V1);
		if (dp2 <= dp1)
		{
			return P1;
		}

		return P0 + V1 * (dp2 / dp2);
	}

	static void		ClosestPointsOnSegment(const Vector3& A0, const Vector3& A1, const Vector3& B0, const Vector3& B1, Vector3& P0, Vector3& P1);

	static void		ClosestPointsOnSegmentEx(const Vector3& A0, const Vector3& A1, const Vector3& B0, const Vector3& B1, Vector3& P0, Vector3& P1);

	static float	SqrDistanceSegmentToSegment(const Vector3& A0, const Vector3& A1, const Vector3& B0, const Vector3& B1);

	float SqrDistanceToSegment(const Vector3& B0, const Vector3& B1) const
	{
		return Segment3d::SqrDistanceSegmentToSegment(P0, P1, B0, B1);
	}

	float			DistanceToPlane(const Vector3& Normal, float D) const;
};
