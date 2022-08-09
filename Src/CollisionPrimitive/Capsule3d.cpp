#include "Capsule3d.h"
#include "AxisAlignedBox3d.h"
#include "Segment3d.h"
#include "../Maths/Maths.h"

bool Capsule3d::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	const Vector3 vx0 = Origin - X0;
	Vector3 Axis = (X1 - X0) / Length;
	const float dp = DotProduct(vx0, Axis);
	if (dp >= -Radius && dp <= Length + Radius)
	{
		const float proj = Clamp<float>(dp, 0, Length);
		const Vector3 proj_pos = Axis * proj;
		const float dist = (vx0 - proj_pos).SquareLength();
		if (dist <= Radius * Radius)
		{
			*t = 0;
			return true;
		}
	}

	const float ad = DotProduct(Axis, Direction);
	const float sqrDist = vx0.SquareLength();
	const float a = 1 - ad * ad;
	const float c = sqrDist - dp * dp - Radius * Radius;

	bool bCheckCaps = false;

	if (c <= 0.f)
	{
		bCheckCaps = true;
	}
	else
	{
		const float b = DotProduct(vx0, Direction) - dp * ad;
		const float delta = b * b - a * c;

		if (delta < 0)
		{
			bCheckCaps = true;
		}
		else
		{
			float time;
			const bool hit = delta < 1e-4;
			if (hit)
			{
				time = (a == 0) ? 0 : (-b / a);

			}
			else
			{
				time = (a == 0) ? 0 : ((-b - sqrtf(delta)) / a);
				if (time < 0)
				{
					return false;
				}
			}

			const Vector3 pos = Origin + Direction * time;
			const float dist = DotProduct(pos - X0, Axis);
			if (dist >= 0 && dist < Length)
			{
				*t = time;
				return true;
			}
			else
			{
				bCheckCaps = !hit;
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
