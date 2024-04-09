#pragma once

#include "../Maths/Vector3.h"

namespace Riemann
{
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

		Vector3		GetDirection() const
		{
			return (P1 - P0).SafeUnit();
		}

		float			SqrDistanceToPoint(const Vector3& Point) const;
		Vector3			ClosestPointToPoint(const Vector3& Point) const;
		static float	SqrDistancePointToSegment(const Vector3& Point, const Vector3& P0, const Vector3& P1);
		static Vector3 	ClosestPointOnSegmentToPoint(const Vector3& Point, const Vector3& P0, const Vector3& P1);
		static void		ClosestPointsBetweenSegments(const Vector3& A0, const Vector3& A1, const Vector3& B0, const Vector3& B1, Vector3& P0, Vector3& P1);
		static void		ClosestPointsBetweenSegmentsEx(const Vector3& A0, const Vector3& A1, const Vector3& B0, const Vector3& B1, Vector3& P0, Vector3& P1);
		static float	SqrDistanceSegmentToSegment(const Vector3& A0, const Vector3& A1, const Vector3& B0, const Vector3& B1);
		float 			SqrDistanceToSegment(const Vector3& B0, const Vector3& B1) const;
		float			DistanceToPlane(const Vector3& Normal, float D) const;
	};
}