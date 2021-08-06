#pragma once

#include "../Maths/Vector3d.h"

class Segment3d
{
public:
	Vector3d	P0;
	Vector3d	P1;

	Segment3d(const Vector3d& _p0, const Vector3d& _p1)
	{
		P0 = _p0;
		P1 = _p1;
	}

	Vector3d		GetDir() const
	{
		return (P1 - P0).SafeUnit();
	}

	float			SqrDistanceToPoint(const Vector3d& Point) const
	{
		Vector3d Closest = Segment3d::ClosestPointOnSegment(P0, P1, Point);
		return (Point - Closest).SquareLength();
	}

	Vector3d		ClosestPointTo(const Vector3d& Point) const
	{
		return Segment3d::ClosestPointOnSegment(P0, P1, Point);
	}

	static float	SqrDistanceSegmentToPoint(const Vector3d& P0, const Vector3d& P1, const Vector3d& Point)
	{
		Vector3d Closest = Segment3d::ClosestPointOnSegment(P0, P1, Point);
		return (Point - Closest).SquareLength();
	}

	static Vector3d ClosestPointOnSegment(const Vector3d& P0, const Vector3d& P1, const Vector3d &Point)
	{
		const Vector3d V1 = P1 - P0;
		const Vector3d V2 = Point - P0;

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

	static void		ClosestPointsOnSegment(const Vector3d& A0, const Vector3d& A1, const Vector3d& B0, const Vector3d& B1, Vector3d& P0, Vector3d& P1);

	static void		ClosestPointsOnSegmentEx(const Vector3d& A0, const Vector3d& A1, const Vector3d& B0, const Vector3d& B1, Vector3d& P0, Vector3d& P1);

	static float	SqrDistanceSegmentToSegment(const Vector3d& A0, const Vector3d& A1, const Vector3d& B0, const Vector3d& B1);

	float SqrDistanceToSegment(const Vector3d& B0, const Vector3d& B1) const
	{
		return Segment3d::SqrDistanceSegmentToSegment(P0, P1, B0, B1);
	}

	float			DistanceToPlane(const Vector3d& Normal, float D) const;
};