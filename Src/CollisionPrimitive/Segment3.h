#pragma once

#include "../Maths/Vector3.h"
#include "Plane3.h"

namespace Riemann
{
	class Segment3
	{
	public:
		Vector3	P0;
		Vector3	P1;

		Segment3(const Vector3& _p0, const Vector3& _p1)
		{
			P0 = _p0;
			P1 = _p1;
		}

		Vector3		GetDirection() const
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

		Vector3		GetCenter() const
		{
			return (P1 + P0) * 0.5f;
		}

		float		SqrDistanceToPoint(const Vector3& Point) const
		{
			Vector3 Closest = Segment3::ClosestPointOnSegmentToPoint(Point, P0, P1);
			return (Point - Closest).SquareLength();
		}

		Vector3		ClosestPointToPoint(const Vector3& Point) const
		{
			return Segment3::ClosestPointOnSegmentToPoint(P0, P1, Point);
		}

		static float	SqrDistancePointToSegment(const Vector3& Point, const Vector3& P0, const Vector3& P1)
		{
			Vector3 Closest = Segment3::ClosestPointOnSegmentToPoint(Point, P0, P1);
			return (Point - Closest).SquareLength();
		}

		static Vector3 	ClosestPointOnSegmentToPoint(const Vector3& Point, const Vector3& P0, const Vector3& P1)
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

		float ProjectUnitRange(const Vector3& Point) const
		{
			Vector3 Center = GetCenter();
			float Extent = GetExtent();
			Vector3 Direction = GetDirection();
			float ProjT = (Point - Center).Dot(Direction);
			float Alpha = ((ProjT / Extent) + 1.0f) * 0.5f;
			return Alpha < 0.0f ? 0.0f : (Alpha > 1.0f ? 1.0f : Alpha);
		}

		static void ClosestPointsBetweenSegments(const Vector3& A0, const Vector3& A1, const Vector3& B0, const Vector3& B1, Vector3& P0, Vector3& P1)
		{
			SolveClosestPointsOnSegment(A0, A1, B0, B1, P0, P1, false);
		}

		static void		ClosestPointsBetweenSegmentsEx(const Vector3& A0, const Vector3& A1, const Vector3& B0, const Vector3& B1, Vector3& P0, Vector3& P1)
		{
			const float kTorSegment = 1e-4f;

			const Vector3 norm0 = (A1 - A0).SafeUnit();
			const Vector3 norm1 = (B1 - B0).SafeUnit();

			const bool bS1IsPoint = norm0.IsZero();
			const bool bS2IsPoint = norm1.IsZero();

			if (bS1IsPoint && bS2IsPoint)
			{
				P0 = A0;
				P1 = B0;
			}
			else if (bS2IsPoint)
			{
				P0 = ClosestPointOnSegmentToPoint(B0, A0, A1);
				P1 = B0;
			}
			else if (bS1IsPoint)
			{
				P0 = A0;
				P1 = ClosestPointOnSegmentToPoint(A0, B0, B1);
			}
			else
			{
				float dp00 = norm0.Dot(norm0);
				float dp11 = norm1.Dot(norm1);
				float dp01 = norm0.Dot(norm1);
				float norm = dp00 * dp11 - dp01 * dp01;
				bool bNearlyParallel = norm < kTorSegment;
				SolveClosestPointsOnSegment(A0, A1, B0, B1, P0, P1, bNearlyParallel);
			}
		}

		static float	SqrDistanceSegmentToSegment(const Vector3& A0, const Vector3& A1, const Vector3& B0, const Vector3& B1)
		{
			if ((B0 - B1).SquareLength() <= 1e-6f)
			{
				return Segment3::SqrDistancePointToSegment(B0, A0, A1);
			}

			if ((A0 - A1).SquareLength() <= 1e-6f)
			{
				return Segment3::SqrDistancePointToSegment(A0, B0, B1);
			}

			Vector3 X, Y;
			ClosestPointsBetweenSegments(A0, A1, B0, B1, X, Y);
			return (X - Y).SquareLength();
		}

		float SqrDistanceToSegment(const Vector3& B0, const Vector3& B1) const
		{
			return Segment3::SqrDistanceSegmentToSegment(P0, P1, B0, B1);
		}

		float DistanceToPlane(const Vector3& Normal, float D) const
		{
			Plane3 plane(Normal, D);
			return plane.DistanceToSegment(P0, P1);
		}

	private:
		static 	void SolveClosestPointsOnSegment(const Vector3& A0, const Vector3& A1, const Vector3& B0, const Vector3& B1, Vector3& P0, Vector3& P1, bool bNearlyParallel)
		{
			const float kTorSegment = 1e-4f;

			Vector3 V0 = A1 - A0;
			Vector3 V1 = B1 - B0;
			Vector3 V2 = A0 - B0;

			const float dp00 = V0.Dot(V0);
			const float dp01 = V0.Dot(V1);
			const float dp02 = V0.Dot(V2);
			const float dp11 = V1.Dot(V1);
			const float dp12 = V1.Dot(V2);

			const float D = dp00 * dp11 - dp01 * dp01;

			float D1 = D;
			float D2 = D;

			float N1;
			float N2;

			if (bNearlyParallel || D < kTorSegment)
			{
				N1 = 0.0f;
				D1 = 1.0f;
				N2 = dp12;
				D2 = dp11;
			}
			else
			{
				N1 = (dp01 * dp12 - dp11 * dp02);
				N2 = (dp00 * dp12 - dp01 * dp02);

				if (N1 < 0.f)
				{
					N1 = 0.f;
					N2 = dp12;
					D2 = dp11;
				}
				else if (N1 > D1)
				{
					N1 = D1;
					N2 = dp12 + dp01;
					D2 = dp11;
				}
			}

			if (N2 < 0.f)
			{
				N2 = 0.f;
				if (-dp02 < 0.f)
				{
					N1 = 0.f;
				}
				else if (-dp02 > dp00)
				{
					N1 = D1;
				}
				else
				{
					N1 = -dp02;
					D1 = dp00;
				}
			}
			else if (N2 > D2)
			{
				N2 = D2;

				if ((-dp02 + dp01) < 0.f)
				{
					N1 = 0.f;
				}
				else if ((-dp02 + dp01) > dp00)
				{
					N1 = D1;
				}
				else
				{
					N1 = (-dp02 + dp01);
					D1 = dp00;
				}
			}

			const float T1 = (fabsf(N1) < kTorSegment ? 0.f : N1 / D1);
			const float T2 = (fabsf(N2) < kTorSegment ? 0.f : N2 / D2);

			P0 = A0 + T1 * V0;
			P1 = B0 + T2 * V1;
		}
	};
}