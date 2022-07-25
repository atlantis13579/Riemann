#include "Capsule3d.h"
#include "AxisAlignedBox3d.h"
#include "Segment3d.h"
#include "../Maths/Maths.h"

bool Capsule3d::IntersectRay(const Vector3& Origin, const Vector3& Dir, float* t) const
{
	const Vector3 X1ToStart = Origin - X0;
	Vector3 Axis = (X1 - X0) / Length;
	const float AxisDotX1ToStart = DotProduct(X1ToStart, Axis);
	if (AxisDotX1ToStart >= -Radius && AxisDotX1ToStart <= Length + Radius)
	{
		const float ClampedProjection = Clamp<float>(AxisDotX1ToStart, 0, Length);
		const Vector3 ClampedProjectionPosition = Axis * ClampedProjection;
		const float Dist2 = (X1ToStart - ClampedProjectionPosition).SquareLength();
		if (Dist2 <= Radius * Radius)
		{
			*t = 0;
			return true;
		}
	}

	// Raycast against cylinder first

	//let <x,y> denote x \dot y
	//cylinder implicit representation: ||((X - x1) \cross Axis)||^2 - R^2 = 0, where X is any point on the cylinder surface (only true because Axis is unit)
	//Using Lagrange's identity we get ||X-x1||^2 ||Axis||^2 - <Axis, X-x1>^2 - R^2 = ||X-x1||^2 - <Axis, X-x1>^2 - R^2 = 0
	//Then plugging the ray into X we have: ||Origin + float Dir - x1||^2 - <Axis, Start + float Dir - x1>^2 - R^2
	// = ||Origin-x1||^2 + t^2 + 2t <Origin-x1, Dir> - <Axis, Origin-x1>^2 - t^2 <Axis,Dir>^2 - 2t<Axis, Origin -x1><Axis, Dir> - R^2 = 0
	//Solving for the quadratic formula we get:
	//a = 1 - <Axis,Dir>^2	Note a = 0 implies Axis and Dir are parallel
	//b = 2(<Origin-x1, Dir> - <Axis, Origin - x1><Axis, Dir>)
	//c = ||Origin-x1||^2 - <Axis, Origin-x1>^2 - R^2 Note this tells us if start point is inside (c < 0) or outside (c > 0) of cylinder

	const float AxisDotX1ToStart2 = AxisDotX1ToStart * AxisDotX1ToStart;
	const float AxisDotDir = DotProduct(Axis, Dir);
	const float AxisDotDir2 = AxisDotDir * AxisDotDir;
	const float X1ToStartDotDir = DotProduct(X1ToStart, Dir);
	const float X1ToStart2 = X1ToStart.SquareLength();
	const float A = 1 - AxisDotDir2;
	const float C = X1ToStart2 - AxisDotX1ToStart2 - Radius * Radius;

	bool bCheckCaps = false;

	if (C <= 0.f)
	{
		// Inside cylinder so check caps
		bCheckCaps = true;
	}
	else
	{
		const float HalfB = (X1ToStartDotDir - AxisDotX1ToStart * AxisDotDir);
		const float QuarterUnderRoot = HalfB * HalfB - A * C;

		if (QuarterUnderRoot < 0)
		{
			bCheckCaps = true;
		}
		else
		{
			float Time;
			const bool bSingleHit = QuarterUnderRoot < 1e-4;
			if (bSingleHit)
			{
				Time = (A == 0) ? 0 : (-HalfB / A);

			}
			else
			{
				Time = (A == 0) ? 0 : ((-HalfB - sqrtf(QuarterUnderRoot)) / A); //we already checked for initial overlap so just take smallest time
				if (Time < 0)	//we must have passed the cylinder
				{
					return false;
				}
			}

			const Vector3 SpherePosition = Origin + Dir * Time;
			const Vector3 CylinderToSpherePosition = SpherePosition - X0;
			const float PositionLengthOnCoreCylinder = DotProduct(CylinderToSpherePosition, Axis);
			if (PositionLengthOnCoreCylinder >= 0 && PositionLengthOnCoreCylinder < Length)
			{
				*t = Time;
				// OutNormal = (CylinderToSpherePosition - Axis * PositionLengthOnCoreCylinder) / R;
				// OutPosition = SpherePosition - OutNormal * Thickness;
				return true;
			}
			else
			{
				//if we have a single hit the ray is tangent to the cylinder.
				//the caps are fully contained in the infinite cylinder, so no need to check them
				bCheckCaps = !bSingleHit;
			}
		}
	}

	if (bCheckCaps)
	{
		//can avoid some work here, but good enough for now
		Sphere3d X1Sphere(X0, Radius);
		Sphere3d X2Sphere(X1, Radius);

		float Time1, Time2;
		Vector3 Position1, Position2;
		Vector3 Normal1, Normal2;
		bool bHitX1 = X1Sphere.IntersectRay(Origin, Dir, &Time1);
		bool bHitX2 = X2Sphere.IntersectRay(Origin, Dir, &Time2);

		if (bHitX1 && bHitX2)
		{
			if (Time1 <= Time2)
			{
				*t = Time1;
				//	OutPosition = Position1;
				//	OutNormal = Normal1;
			}
			else
			{
				*t = Time2;
				//	OutPosition = Position2;
				//	OutNormal = Normal2;
			}

			return true;
		}
		else if (bHitX1)
		{
			*t = Time1;
			// OutPosition = Position1;
			// OutNormal = Normal1;
			return true;
		}
		else if (bHitX2)
		{
			*t = Time2;
			// OutPosition = Position2;
			// OutNormal = Normal2;
			return true;
		}
	}

	return false;
}

bool Capsule3d::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
{
	AxisAlignedBox3d aabb(Bmin, Bmax);
	return aabb.SqrDistanceToSegment(X0, X1) <= Radius * Radius;
}

bool Capsule3d::IntersectSphere(const Vector3& InCenter, float InRadius) const
{
	float SqrDist = Segment3d::SqrDistancePointToSegment(InCenter, X0, X1);
	return SqrDist <= (Radius + InRadius) * (Radius + InRadius);
}

bool Capsule3d::IntersectCapsule(const Vector3& P0, const Vector3& P1, float rRadius) const
{
	if ((P1 - P0).SquareLength() < TINY_NUMBER)
	{
		return IntersectSphere(P0, rRadius);
	}

	const float SqrDist = Segment3d::SqrDistanceSegmentToSegment(P0, P1, X0, X1);
	return SqrDist <= (Radius + rRadius) * (Radius + rRadius);
}
