#include "Capsule3d.h"
#include "AxisAlignedBox3d.h"
#include "Segment3d.h"
#include "../Maths/Maths.h"

bool Capsule3d::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	const Vector3 VX1 = Origin - X0;
	Vector3 Axis = (X1 - X0) / Length;
	const float dp = DotProduct(VX1, Axis);
	if (dp >= -Radius && dp <= Length + Radius)
	{
		const float proj = Clamp<float>(dp, 0, Length);
		const Vector3 proj_pos = Axis * proj;
		const float dist = (VX1 - proj_pos).SquareLength();
		if (dist <= Radius * Radius)
		{
			*t = 0;
			return true;
		}
	}

	const float AxisDotDir = DotProduct(Axis, Direction);
	const float AxisDotDir2 = AxisDotDir * AxisDotDir;
	const float X1ToStartDotDir = DotProduct(VX1, Direction);
	const float X1ToStart2 = VX1.SquareLength();
	const float A = 1 - AxisDotDir2;
	const float C = X1ToStart2 - dp * dp - Radius * Radius;

	bool bCheckCaps = false;

	if (C <= 0.f)
	{
		// Inside cylinder so check caps
		bCheckCaps = true;
	}
	else
	{
		const float HalfB = (X1ToStartDotDir - dp * AxisDotDir);
		const float QuarterUnderRoot = HalfB * HalfB - A * C;

		if (QuarterUnderRoot < 0)
		{
			bCheckCaps = true;
		}
		else
		{
			float time;
			const bool bSingleHit = QuarterUnderRoot < 1e-4;
			if (bSingleHit)
			{
				time = (A == 0) ? 0 : (-HalfB / A);

			}
			else
			{
				time = (A == 0) ? 0 : ((-HalfB - sqrtf(QuarterUnderRoot)) / A); //we already checked for initial overlap so just take smallest time
				if (time < 0)	//we must have passed the cylinder
				{
					return false;
				}
			}

			const Vector3 SpherePosition = Origin + Direction * time;
			const Vector3 CylinderToSpherePosition = SpherePosition - X0;
			const float PositionLengthOnCoreCylinder = DotProduct(CylinderToSpherePosition, Axis);
			if (PositionLengthOnCoreCylinder >= 0 && PositionLengthOnCoreCylinder < Length)
			{
				*t = time;
				return true;
			}
			else
			{
				bCheckCaps = !bSingleHit;
			}
		}
	}

	if (bCheckCaps)
	{
		Sphere3d sphere1(X0, Radius);
		Sphere3d sphere2(X1, Radius);

		float t1, t2;
		bool hit1 = sphere1.IntersectRay(Origin, Direction, &t1);
		bool hit2 = sphere2.IntersectRay(Origin, Direction, &t2);

		if (hit1 && hit2)
		{
			*t = std::min(t1, t2);
			return true;
		}
		else if (hit1)
		{
			*t = t1;
			return true;
		}
		else if (hit2)
		{
			*t = t2;
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

bool Capsule3d::IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
{
	assert(false);
	return false;
}
