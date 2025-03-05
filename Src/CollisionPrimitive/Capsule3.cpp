#include "Capsule3.h"
#include "AxisAlignedBox3.h"
#include "Sphere3.h"
#include "Segment3.h"
#include "ConvexMesh.h"
#include "HeightField3.h"
#include "TriangleMesh.h"

namespace Riemann
{

	bool Capsule3::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		const Vector3 vx0 = Origin - X0;
		Vector3 Axis = (X1 - X0) / Length;
		const float dp = DotProduct(vx0, Axis);
		if (dp >= -Radius && dp <= Length + Radius)
		{
			const float proj = Maths::Clamp<float>(dp, 0, Length);
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
			Sphere3 sphere1(X0, Radius);
			Sphere3 sphere2(X1, Radius);

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

bool Capsule3::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
{
	AxisAlignedBox3 aabb(Bmin, Bmax);
	return aabb.SqrDistanceToSegment(X0, X1) <= Radius * Radius;
}

bool Capsule3::IntersectSphere(const Vector3& InCenter, float InRadius) const
{
	float SqrDist = Segment3::SqrDistancePointToSegment(InCenter, X0, X1);
	return SqrDist <= (Radius + InRadius) * (Radius + InRadius);
}

bool Capsule3::IntersectSegment(const Vector3& P0, const Vector3& P1) const
{
	if ((P1 - P0).SquareLength() < TINY_NUMBER)
	{
		float SqrDist = Segment3::SqrDistancePointToSegment(P0, X0, X1);
		return SqrDist <= Radius * Radius;
	}

	const float SqrDist = Segment3::SqrDistanceSegmentToSegment(P0, P1, X0, X1);
	return SqrDist <= Radius * Radius;
}

bool Capsule3::IntersectCapsule(const Vector3& P0, const Vector3& P1, float rRadius) const
{
	if ((P1 - P0).SquareLength() < TINY_NUMBER)
	{
		return IntersectSphere(P0, rRadius);
	}

	const float SqrDist = Segment3::SqrDistanceSegmentToSegment(P0, P1, X0, X1);
	return SqrDist <= (Radius + rRadius) * (Radius + rRadius);
}

// computes shortest vector going from capsule axis to triangle edge
static Vector3 computeEdgeAxis(const Vector3& p, const Vector3& a, const Vector3& q, const Vector3& b)
{
	const Vector3 T = q - p;
	const float ADotA = a.Dot(a);
	const float ADotB = a.Dot(b);
	const float ADotT = a.Dot(T);
	const float BDotT = b.Dot(T);
	const float BDotB = b.Dot(b);

	const float denom = ADotA * BDotB - ADotB * ADotB;

	float t = fabsf(denom) < 1e-6f ? 0.0f : (ADotT * BDotB - BDotT * ADotB) / denom;
	t = Maths::Clamp(t, 0.0f, 1.0f);

	float u = fabsf(BDotB) < 1e-6f ? 0.0f : (t * ADotB - BDotT) / BDotB;

	if (u < 0.0f)
	{
		u = 0.0f;
		t = ADotT / ADotA;
		t = Maths::Clamp(t, 0.0f, 1.0f);
	}
	else if (u > 1.0f)
	{
		u = 1.0f;
		t = (ADotB + ADotT) / ADotA;
		t = Maths::Clamp(t, 0.0f, 1.0f);
	}
	return T + b * u - a * t;
}

bool Capsule3::IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
{
	const float sqrDist = Segment3::SqrDistancePointToSegment(A, X0, X1);
	if (sqrDist <= Radius * Radius)
		return true;

	Vector3 N = (A - B).Cross(A - C);
	Vector3 Axis = X1 - X0;

	if (!intersectAxis(*this, A, B, C, N))
		return false;

	if (!intersectAxis(*this, A, B, C, computeEdgeAxis(A, B - A, X0, Axis)))
		return false;

	if (!intersectAxis(*this, A, B, C, computeEdgeAxis(B, C - B, X0, Axis)))
		return false;

	if (!intersectAxis(*this, A, B, C, computeEdgeAxis(C, A - C, X0, Axis)))
		return false;

	return true;
}

bool Capsule3::PenetrateSphere(const Vector3& rCenter, float rRadius, Vector3* Normal, float* Depth) const
{
	const float s = rRadius + Radius;
	Vector3 Closest = Segment3::ClosestPointOnSegmentToPoint(rCenter, X0, X1);
	if ((rCenter - Closest).SquareLength() > s * s)
	{
		return false;
	}

	*Normal = rCenter - Closest;

	const float len = Normal->Length();
	if (len < 1e-6f)
	{
		*Normal = Vector3::UnitY();
	}
	else
	{
		*Normal /= len;
	}
	*Normal = -*Normal;
	*Depth = s - len;
	return true;
}

bool Capsule3::PenetrateCapsule(const Vector3& P0, const Vector3& P1, float rRadius, Vector3* Normal, float* Depth) const
{
	Vector3 s, t;
	Segment3::ClosestPointsBetweenSegmentsEx(X0, X1, P0, P1, s, t);

	const float r2 = Radius + rRadius;

	if ((s - t).SquareLength() > r2 * r2)
	{
		return false;
	}

	*Normal = s - t;

	const float len = Normal->Length();
	if (len < 1e-6f)
	{
		*Normal = Vector3::UnitY();
	}
	else
	{
		*Normal /= len;
	}
	*Depth = r2 - len;
	return true;
}

bool Capsule3::SweepAABB(const Vector3& Direction, const Vector3& bmin, const Vector3& bmax, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Capsule3::SweepSphere(const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Capsule3::SweepPlane(const Vector3& Direction, const Vector3& Normal, float D, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Capsule3::SweepCapsule(const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Capsule3::SweepConvex(const Vector3& Direction, const ConvexMesh* convex, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Capsule3::SweepHeightField(const Vector3& Direction, const HeightField3* hf, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Capsule3::SweepTriangleMesh(const Vector3& Direction, const TriangleMesh* trimesh, Vector3* n, float* t) const
{
	// TODO
	return false;
}


}
