#include "AxisAlignedBox3.h"
#include "Capsule3.h"
#include "Cylinder3.h"
#include "Sphere3.h"
#include "Quad3.h"
#include "Segment3.h"
#include "Triangle3.h"
#include "ConvexMesh.h"
#include "HeightField3.h"
#include "TriangleMesh.h"
#include "GJK.h"

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

bool Capsule3::SweepAABB(const Vector3& Direction, const Vector3& bmin, const Vector3& bmax, Vector3* p, Vector3* n, float* t) const
{
	if (IntersectAABB(bmin, bmax))
	{
		if (p)
		{
			*p = GetCenter();
		}
		if (n)
		{
			*n = -Direction;
			n->SafeNormalize();
		}
		if (t)
		{
			*t = 0.0f;
		}
		return true;
	}

	AxisAlignedBox3 box(bmin, bmax);
	GJKShapecast gjk;
	return gjk.Solve(Direction, this, &box, p, n, t);
}

bool Capsule3::SweepSphere(const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* p, Vector3* n, float* t) const
{
	if (IntersectSphere(rCenter, rRadius))
	{
		if (p)
		{
			*p = GetCenter();
		}
		if (n)
		{
			*n = -Direction;
			n->SafeNormalize();
		}
		if (t)
		{
			*t = 0.0f;
		}
		return true;
	}

	Sphere3 sp(rCenter, rRadius);
	GJKShapecast gjk;
	return gjk.Solve(Direction, this, &sp, p, n, t);
}

bool Capsule3::SweepPlane(const Vector3& Direction, const Vector3& Normal, float D, Vector3* p, Vector3* n, float* t) const
{
	Plane3 plane(Normal, D);
    
    const Vector3 Origin = GetCenter();
	const float dp = Direction.Dot(Normal);
	if (fabsf(dp) < 1e-6f)
	{
		if (plane.IntersectCapsule(X0, X1, Radius))
		{
			*n = -Direction;
			*t = 0.0f;
			return true;
		}
	}

	const Vector3 RelativeOrigin = GetSupport(Direction);
	if (plane.IntersectRay(RelativeOrigin, Direction, t))
	{
		*n = dp < 0.0f ? Normal : -Normal;
		return true;
	}
	return false;
}

bool Capsule3::SweepCylinder(const Vector3& Direction, const Vector3& _X0, const Vector3& _X1, float _Radius, Vector3* p, Vector3* n, float* t) const
{
    Cylinder3 cylinder(_X0, _X1, _Radius);
	GJKIntersection gjkIntersect;
	if (gjkIntersect.Solve(this, &cylinder) == GJK_status::Intersect)
	{
		if (p)
		{
			*p = GetCenter();
		}
		if (n)
		{
			*n = -Direction;
			n->SafeNormalize();
		}
		if (t)
		{
			*t = 0.0f;
		}
		return true;
	}

    GJKShapecast gjk;
    return gjk.Solve(Direction, this, &cylinder, p, n, t);
}

static void computeEdgeEdgeDist(const Vector3& p, const Vector3& a, const Vector3& q, const Vector3& b, Vector3* x, Vector3* y)
{
    const Vector3 T = q - p;
    const float ADotA = a.Dot(a);
    const float BDotB = b.Dot(b);
    const float ADotB = a.Dot(b);
    const float ADotT = a.Dot(T);
    const float BDotT = b.Dot(T);

    // t parameterizes ray (p, a)
    // u parameterizes ray (q, b)
    // Compute t for the closest point on ray (p, a) to ray (q, b)
    const float Denom = ADotA * BDotB - ADotB * ADotB;

    float t;
    if (Denom != 0.0f)
    {
        t = (ADotT * BDotB - BDotT * ADotB) / Denom;

        // Clamp result so t is on the segment (p, a)
        if (t < 0.0f)
            t = 0.0f;
        else if (t > 1.0f)
            t = 1.0f;
    }
    else
    {
        t = 0.0f;
    }

    // find u for point on ray (q, b) closest to point at t
    float u;
    if (BDotB != 0.0f)
    {
        u = (t * ADotB - BDotT) / BDotB;
        if (u < 0.0f)
        {
            u = 0.0f;
            if (ADotA != 0.0f)
            {
                t = ADotT / ADotA;
                if (t < 0.0f)
                    t = 0.0f;
                else if (t > 1.0f)
                    t = 1.0f;
            }
            else
            {
                t = 0.0f;
            }
        }
        else if (u > 1.0f)
        {
            u = 1.0f;
            if (ADotA != 0.0f)
            {
                t = (ADotB + ADotT) / ADotA;
                if (t < 0.0f)
                    t = 0.0f;
                else if (t > 1.0f)
                    t = 1.0f;
            }
            else
            {
                t = 0.0f;
            }
        }
    }
    else
    {
        u = 0.0f;
        if (ADotA != 0.0f)
        {
            t = ADotT / ADotA;
            if (t < 0.0f)
                t = 0.0f;
            else if (t > 1.0f)
                t = 1.0f;
        }
        else
        {
            t = 0.0f;
        }
    }

    *x = p + a * t;
    *y = q + b * u;
}

bool Capsule3::SweepCapsule(const Vector3& Direction, const Vector3& _X0, const Vector3& _X1, float _Radius, Vector3* p, Vector3* n, float* t) const
{
    const float r2 = Radius + _Radius;

    const bool testInitialOverlap = true;
    if(testInitialOverlap)
    {
        float dist;
        if ((X0 - X1).IsZero())
        {
            dist = Segment3::SqrDistancePointToSegment(X0, _X0, _X1) < r2*  r2;
        }
        else if ((_X0 - _X1).IsZero())
        {
            dist = Segment3::SqrDistancePointToSegment(_X0, X0, X1) < r2 * r2;
        }
        else
        {
            dist = Segment3::SqrDistanceSegmentToSegment(X0, X1, _X0, _X1) < r2 * r2;
        }

        assert(dist >= 0.0f);
        if (dist > 1e-6f)
        {
            *t = 0.0f;
            *n = -Direction;
            return true;
        }
    }
    
    // 1. Extrude capsule0 by capsule1's length
    // 2. Inflate extruded shape by capsule1's radius
    // 3. Raycast against resulting shape

    const Vector3 D = (_X1 - _X0) * 0.5f;
    const Vector3 p0 = X0 - D;
    const Vector3 p1 = X1 - D;
    const Vector3 p0b = X0 + D;
    const Vector3 p1b = X1 + D;

    Vector3 Normal = Triangle3::CalculateNormal(p0b, p1b, p1, false);
    const float sweepDist = FLT_MAX;
    float minDist =  sweepDist;
    bool success = false;

    Vector3 pa,pb,pc;
    if ((Normal.Dot(Direction)) >= 0)  // Same direction
    {
        Normal *= r2;
        pc = p0 - Normal;
        pa = p1 - Normal;
        pb = p1b - Normal;
    }
    else
    {
        Normal *= r2;
        pb = p0 + Normal;
        pa = p1 + Normal;
        pc = p1b + Normal;
    }
    
    float tt, uu, vv;
    const Vector3 center = GetCenter();
    if (Quad3::RayIntersectQuad(center, Direction, pa, pb, pc, &tt, &uu, &vv) && tt >= 0.0f && tt < minDist)
    {
        minDist = tt;
        success = true;
    }
    
    if (!success)
    {
        Capsule3 capsules[4];
        capsules[0] = Capsule3(p0, p1, r2);
        capsules[1] = Capsule3(p1, p1b, r2);
        capsules[2] = Capsule3(p1b, p0b, r2);
        capsules[3] = Capsule3(p0, p0b, r2);

        for (int i=0; i<4; i++)
        {
            float w;
            if (capsules[i].IntersectRay(center, Direction, &w))
            {
                if (w >= 0.0f && w <= minDist)
                {
                    minDist = w;
                    success = true;
                }
            }
        }
    }
    
    if (success)
    {
        const Vector3 p00 = X0 - minDist * Direction;
        const Vector3 p01 = X1 - minDist * Direction;

        const Vector3 edge0 = p01 - p00;
        const Vector3 edge1 = (_X1 - _X0);

        Vector3 x, y;
        computeEdgeEdgeDist(p00, edge0, _X0, edge1, &x, &y);

        *n = (x - y);
        const float epsilon = 0.001f;
        if (n->Normalize() < epsilon)
        {
            *n = edge1.Cross(edge0);
            if (n->Normalize() < epsilon)
            {
                computeEdgeEdgeDist(X0, X1 - X0, _X0, edge1, &x, &y);
                *n = (x - y);
                n->Normalize();
            }
        }

		*t = minDist;
        *p = (_Radius * x + Radius * y) / r2;
		return true;
    }

    return success;
}

bool Capsule3::SweepConvex(const Vector3& Direction, const ConvexMesh* convex, Vector3* p, Vector3* n, float* t) const
{
	if (convex)
	{
		GJKIntersection gjkIntersect;
		if (gjkIntersect.Solve(this, convex) == GJK_status::Intersect)
		{
			if (p)
			{
				*p = GetCenter();
			}
			if (n)
			{
				*n = -Direction;
				n->SafeNormalize();
			}
			if (t)
			{
				*t = 0.0f;
			}
			return true;
		}
	}

	GJKShapecast gjk;
	return gjk.Solve(Direction, this, convex, p, n, t);
}

static bool CapsuleSweepMovingAABBAABBInterval(const Vector3& movingMin, const Vector3& movingMax, const Vector3& direction, const Vector3& staticMin, const Vector3& staticMax, float maxDist, float& enter, float& exit)
{
	Box3 movingBox(movingMin, movingMax);
	if (movingBox.Intersect(staticMin, staticMax))
	{
		enter = 0.0f;
		exit = maxDist;
		return true;
	}

	const Vector3 center = (movingMin + movingMax) * 0.5f;
	const Vector3 extents = (movingMax - movingMin) * 0.5f;
	const Vector3 expandedMin = staticMin - extents;
	const Vector3 expandedMax = staticMax + extents;

	enter = 0.0f;
	exit = maxDist;
	for (int i = 0; i < 3; ++i)
	{
		if (fabsf(direction[i]) < 1.0e-8f)
		{
			if (center[i] < expandedMin[i] || center[i] > expandedMax[i])
			{
				return false;
			}
			continue;
		}

		const float invDir = 1.0f / direction[i];
		float t0 = (expandedMin[i] - center[i]) * invDir;
		float t1 = (expandedMax[i] - center[i]) * invDir;
		if (t0 > t1)
		{
			std::swap(t0, t1);
		}

		enter = std::max(enter, t0);
		exit = std::min(exit, t1);
		if (enter > exit)
		{
			return false;
		}
	}

	if (exit < 0.0f || enter > maxDist)
	{
		return false;
	}

	enter = std::max(enter, 0.0f);
	return true;
}

static bool CapsuleSweepMovingAABBAABB(const Vector3& movingMin, const Vector3& movingMax, const Vector3& direction, const Vector3& staticMin, const Vector3& staticMax, float maxDist, float& toi)
{
	float enter, exit;
	if (!CapsuleSweepMovingAABBAABBInterval(movingMin, movingMax, direction, staticMin, staticMax, maxDist, enter, exit))
	{
		return false;
	}
	toi = std::max(enter, 0.0f);
	return true;
}

static void ClosestSegmentTrianglePoints(const Vector3& s0, const Vector3& s1, const Vector3& A, const Vector3& B, const Vector3& C, Vector3& pointOnSegment, Vector3& pointOnTriangle)
{
	pointOnSegment = s0;
	pointOnTriangle = Triangle3::ClosestPointOnTriangleToPoint(s0, A, B, C);
	float bestSqr = (pointOnSegment - pointOnTriangle).SquareLength();

	const Vector3 triFromS1 = Triangle3::ClosestPointOnTriangleToPoint(s1, A, B, C);
	float sqr = (s1 - triFromS1).SquareLength();
	if (sqr < bestSqr)
	{
		bestSqr = sqr;
		pointOnSegment = s1;
		pointOnTriangle = triFromS1;
	}

	const Vector3 edges[3][2] =
	{
		{ A, B },
		{ B, C },
		{ C, A },
	};
	for (int i = 0; i < 3; ++i)
	{
		Vector3 segPoint, edgePoint;
		Segment3::ClosestPointsBetweenSegmentsEx(s0, s1, edges[i][0], edges[i][1], segPoint, edgePoint);
		sqr = (segPoint - edgePoint).SquareLength();
		if (sqr < bestSqr)
		{
			bestSqr = sqr;
			pointOnSegment = segPoint;
			pointOnTriangle = edgePoint;
		}
	}
}

static void ComputeCapsuleTriangleSweepResult(const Capsule3& capsule, const Vector3& direction, float hitTime, const Vector3& A, const Vector3& B, const Vector3& C, Vector3* p, Vector3* n, float* t)
{
	if (t)
	{
		*t = hitTime;
	}

	const Vector3 movedX0 = capsule.X0 + direction * hitTime;
	const Vector3 movedX1 = capsule.X1 + direction * hitTime;
	Vector3 pointOnSegment, pointOnTriangle;
	ClosestSegmentTrianglePoints(movedX0, movedX1, A, B, C, pointOnSegment, pointOnTriangle);

	if (p)
	{
		*p = pointOnTriangle;
	}
	if (n)
	{
		*n = pointOnSegment - pointOnTriangle;
		if (n->SafeNormalize() <= 1.0e-4f)
		{
			*n = Triangle3::CalculateNormal(A, B, C, false);
			if (n->SafeNormalize() <= 1.0e-6f)
			{
				*n = -direction;
				n->SafeNormalize();
			}
		}
		if (n->Dot(direction) > 0.0f)
		{
			*n = -*n;
		}
	}
}

static void AddExtrudedTriangle(Vector3 outTris[21], int& numVerts, const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& direction)
{
	Vector3 n = (b - a).Cross(c - a);
	if (n.Dot(direction) > 0.0f)
	{
		outTris[numVerts++] = a;
		outTris[numVerts++] = c;
		outTris[numVerts++] = b;
	}
	else
	{
		outTris[numVerts++] = a;
		outTris[numVerts++] = b;
		outTris[numVerts++] = c;
	}
}

static bool SweepCapsuleTriangleTOI(const Capsule3& capsule, const Vector3& direction, const Vector3& A, const Vector3& B, const Vector3& C, float maxDistance, float& hitTime)
{
	if (direction.SquareLength() <= 1.0e-12f)
	{
		return false;
	}

	if (capsule.IntersectTriangle(A, B, C))
	{
		hitTime = 0.0f;
		return true;
	}

	const Vector3 unitDir = direction.SafeUnit();
	const Vector3 extrusionDir = (capsule.X0 - capsule.X1) * 0.5f;
	const float halfHeight = extrusionDir.Length();
	if (halfHeight <= 1.0e-6f)
	{
		Sphere3 sphere(capsule.X0, capsule.Radius);
		Vector3 unusedP, unusedN;
		return sphere.SweepTriangle(direction, A, B, C, &unusedP, &unusedN, &hitTime) && hitTime <= maxDistance;
	}

	const Vector3 capsuleAxis = extrusionDir / halfHeight;
	if (fabsf(capsuleAxis.Dot(unitDir)) >= 1.0f - 1.0e-5f)
	{
		const Vector3 sphereCenter = capsule.GetCenter() + unitDir * halfHeight;
		Sphere3 sphere(sphereCenter, capsule.Radius);
		Vector3 unusedP, unusedN;
		return sphere.SweepTriangle(direction, A, B, C, &unusedP, &unusedN, &hitTime) && hitTime <= maxDistance;
	}

	Vector3 extrudedTris[21];
	int numVerts = 0;

	const Vector3 triNormal = (B - A).Cross(C - A);
	const Vector3 p0 = A - extrusionDir;
	const Vector3 p1 = B - extrusionDir;
	const Vector3 p2 = C - extrusionDir;
	const Vector3 p0b = A + extrusionDir;
	const Vector3 p1b = B + extrusionDir;
	const Vector3 p2b = C + extrusionDir;

	if (triNormal.Dot(extrusionDir) >= 0.0f)
	{
		AddExtrudedTriangle(extrudedTris, numVerts, p0b, p1b, p2b, direction);
	}
	else
	{
		AddExtrudedTriangle(extrudedTris, numVerts, p0, p1, p2, direction);
	}

	AddExtrudedTriangle(extrudedTris, numVerts, p1, p1b, p2b, direction);
	AddExtrudedTriangle(extrudedTris, numVerts, p1, p2b, p2, direction);
	AddExtrudedTriangle(extrudedTris, numVerts, p0, p2, p2b, direction);
	AddExtrudedTriangle(extrudedTris, numVerts, p0, p2b, p0b, direction);
	AddExtrudedTriangle(extrudedTris, numVerts, p0b, p1b, p1, direction);
	AddExtrudedTriangle(extrudedTris, numVerts, p0b, p1, p0, direction);

	Sphere3 sphere(capsule.GetCenter(), capsule.Radius);
	bool hit = false;
	float bestDistance = maxDistance;
	for (int i = 0; i < numVerts; i += 3)
	{
		Vector3 unusedP, unusedN;
		float currentDistance;
		if (sphere.SweepTriangle(direction, extrudedTris[i], extrudedTris[i + 1], extrudedTris[i + 2], &unusedP, &unusedN, &currentDistance) && currentDistance <= bestDistance)
		{
			bestDistance = currentDistance;
			hit = true;
		}
	}

	if (!hit)
	{
		return false;
	}

	hitTime = bestDistance;
	return true;
}

static void TestCapsuleTriangleForBestSweep(const Capsule3& capsule, const Vector3& direction, const Vector3& unitDir, const Vector3& A, const Vector3& B, const Vector3& C, float& bestDistance, float& bestAlignment, Vector3 bestTri[3], bool& hit)
{
	float currentDistance;
	if (!SweepCapsuleTriangleTOI(capsule, direction, A, B, C, bestDistance, currentDistance))
	{
		return;
	}

	Vector3 triNormal = (B - A).Cross(C - A);
	if (triNormal.SafeNormalize() <= 1.0e-6f)
	{
		triNormal = -unitDir;
	}

	const float alignment = Triangle3::ComputeAlignmentValue(triNormal, unitDir);
	if (!hit || Triangle3::IsBetterTriangle(currentDistance, alignment, bestDistance, bestAlignment))
	{
		bestDistance = currentDistance;
		bestAlignment = alignment;
		bestTri[0] = A;
		bestTri[1] = B;
		bestTri[2] = C;
		hit = true;
	}
}

bool Capsule3::SweepTriangle(const Vector3& Direction, const Vector3 &A, const Vector3 &B, const Vector3 &C, Vector3* p, Vector3* n, float* t) const
{
	float hitTime;
	if (!SweepCapsuleTriangleTOI(*this, Direction, A, B, C, FLT_MAX, hitTime))
	{
		return false;
	}

	ComputeCapsuleTriangleSweepResult(*this, Direction, hitTime, A, B, C, p, n, t);
	return true;
}

bool Capsule3::SweepHeightField(const Vector3& Direction, const HeightField3* hf, Vector3* p, Vector3* n, float* t) const
{
	if (hf == nullptr || hf->Cells == nullptr || hf->nX < 2 || hf->nZ < 2 || Direction.SquareLength() <= 1.0e-12f)
	{
		return false;
	}

	const Box3 movingBounds = CalculateBoundingVolume();

	float hfEnter, hfExit;
	if (!CapsuleSweepMovingAABBAABBInterval(movingBounds.Min, movingBounds.Max, Direction, hf->BV.Min, hf->BV.Max, FLT_MAX, hfEnter, hfExit))
	{
		return false;
	}

	Box3 overlap;
	if (hfExit == FLT_MAX)
	{
		overlap = hf->BV;
	}
	else
	{
		Box3 sweptBox(movingBounds.Min + Direction * hfEnter, movingBounds.Max + Direction * hfEnter);
		sweptBox.Encapsulate(movingBounds.Min + Direction * hfExit);
		sweptBox.Encapsulate(movingBounds.Max + Direction * hfExit);
		if (!sweptBox.GetIntersection(hf->BV, overlap))
		{
			return false;
		}
	}

	const int i0 = std::max(0, std::min((int)hf->nX - 2, (int)((overlap.Min.x - hf->BV.Min.x) * hf->InvDX)));
	const int j0 = std::max(0, std::min((int)hf->nZ - 2, (int)((overlap.Min.z - hf->BV.Min.z) * hf->InvDZ)));
	const int i1 = std::max(0, std::min((int)hf->nX - 2, (int)((overlap.Max.x - hf->BV.Min.x) * hf->InvDX)));
	const int j1 = std::max(0, std::min((int)hf->nZ - 2, (int)((overlap.Max.z - hf->BV.Min.z) * hf->InvDZ)));

	bool hit = false;
	float bestDistance = FLT_MAX;
	float bestAlignment = 2.0f;
	Vector3 bestTri[3];
	const Vector3 unitDir = Direction.SafeUnit();

	for (int i = i0; i <= i1; ++i)
	{
		for (int j = j0; j <= j1; ++j)
		{
			Box3 cellBox;
			if (!hf->GetCellBV(i, j, cellBox))
			{
				continue;
			}

			float cellToi;
			if (!CapsuleSweepMovingAABBAABB(movingBounds.Min, movingBounds.Max, Direction, cellBox.Min, cellBox.Max, bestDistance, cellToi))
			{
				continue;
			}

			Vector3 tris[6];
			const int numTriVerts = hf->GetCellTriangle(i, j, tris);
			for (int k = 0; k < numTriVerts; k += 3)
			{
				TestCapsuleTriangleForBestSweep(*this, Direction, unitDir, tris[k], tris[k + 1], tris[k + 2], bestDistance, bestAlignment, bestTri, hit);
			}
		}
	}

	if (!hit)
	{
		return false;
	}

	ComputeCapsuleTriangleSweepResult(*this, Direction, bestDistance, bestTri[0], bestTri[1], bestTri[2], p, n, t);
	return true;
}

bool Capsule3::SweepTriangleMesh(const Vector3& Direction, const TriangleMesh* trimesh, Vector3* p, Vector3* n, float* t) const
{
	if (trimesh == nullptr || trimesh->NumTriangles == 0 || Direction.SquareLength() <= 1.0e-12f)
	{
		return false;
	}

	const Box3 movingBounds = CalculateBoundingVolume();

	float meshToi;
	if (!CapsuleSweepMovingAABBAABB(movingBounds.Min, movingBounds.Max, Direction, trimesh->BoundingVolume.Min, trimesh->BoundingVolume.Max, FLT_MAX, meshToi))
	{
		return false;
	}

	bool hit = false;
	float bestDistance = FLT_MAX;
	float bestAlignment = 2.0f;
	Vector3 bestTri[3];
	const Vector3 unitDir = Direction.SafeUnit();

	for (uint32_t i = 0; i < trimesh->NumTriangles; ++i)
	{
		const Vector3 A = trimesh->GetVertex(i, 0);
		const Vector3 B = trimesh->GetVertex(i, 1);
		const Vector3 C = trimesh->GetVertex(i, 2);
		const Box3 triBounds(A, B, C);

		float triToi;
		if (!CapsuleSweepMovingAABBAABB(movingBounds.Min, movingBounds.Max, Direction, triBounds.Min, triBounds.Max, bestDistance, triToi))
		{
			continue;
		}

		TestCapsuleTriangleForBestSweep(*this, Direction, unitDir, A, B, C, bestDistance, bestAlignment, bestTri, hit);
	}

	if (!hit)
	{
		return false;
	}

	ComputeCapsuleTriangleSweepResult(*this, Direction, bestDistance, bestTri[0], bestTri[1], bestTri[2], p, n, t);
	return true;
}

int Capsule3::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
{
	Vector3 DirectionXZ = Vector3(Direction.x, 0.0f, Direction.z);

	// hit  top/bottom
	float len = DirectionXZ.Length();
	if (len == 0.0f)
	{
		FacePoints[0] = GetSupport(Direction);
		return 1;
	}

	Vector3 support = (Radius / len) * DirectionXZ;
	Vector3 support_top = Vector3(0, Length * 0.5f, 0) - support;
	Vector3 support_bottom = Vector3(0, -Length * 0.5f, 0) - support;

	float proj_top = support_top.Dot(Direction);
	float proj_bottom = support_bottom.Dot(Direction);

	// near parallel, hit edge
	if (fabsf(proj_top - proj_bottom) < 0.02f * Direction.Length())
	{
		FacePoints[0] = support_top;
		FacePoints[1] = support_bottom;
		return 2;
	}

	FacePoints[0] = GetSupport(Direction);
	return 1;
}


void Capsule3::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
	const int stackCount = 5;
	const int sliceCount = 8;

	GetVertices(stackCount, sliceCount, Vertices, &Normals);

	for (int i = 1; i <= sliceCount; i++)
	{
		Indices.push_back(0);
		Indices.push_back(i + 1);
		Indices.push_back(i);
	}

	int baseIndex = 1;
	int Count = sliceCount + 1;
	for (int i = 0; i < stackCount - 2; i++)
	{
		for (int j = 0; j < sliceCount; j++)
		{
			Indices.push_back(baseIndex + i * Count + j);
			Indices.push_back(baseIndex + i * Count + j + 1);
			Indices.push_back(baseIndex + (i + 1) * Count + j);

			Indices.push_back(baseIndex + (i + 1) * Count + j);
			Indices.push_back(baseIndex + i * Count + j + 1);
			Indices.push_back(baseIndex + (i + 1) * Count + j + 1);
		}
	}
	int PoleIndex = (int)Vertices.size() - 1;
	baseIndex = PoleIndex - Count;
	for (int i = 0; i < sliceCount; i++)
	{
		Indices.push_back(PoleIndex);
		Indices.push_back(baseIndex + i);
		Indices.push_back(baseIndex + i + 1);
	}
}


void Capsule3::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
{
	const int stackCount = 5;
	const int sliceCount = 8;

	GetVertices(stackCount, sliceCount, Vertices, nullptr);

	for (int i = 1; i <= sliceCount; i++)
	{
		Indices.push_back(0);
		Indices.push_back(i);

		Indices.push_back(i);
		Indices.push_back(i + 1);
	}

	int baseIndex = 1;
	int Count = sliceCount + 1;
	for (int i = 0; i < stackCount - 2; i++)
	{
		for (int j = 0; j < sliceCount; j++)
		{
			Indices.push_back(baseIndex + i * Count + j);
			Indices.push_back(baseIndex + i * Count + j + 1);

			Indices.push_back(baseIndex + i * Count + j + 1);
			Indices.push_back(baseIndex + (i + 1) * Count + j + 1);

			Indices.push_back(baseIndex + (i + 1) * Count + j + 1);
			Indices.push_back(baseIndex + i * Count + j + 1);

			Indices.push_back(baseIndex + i * Count + j + 1);
			Indices.push_back(baseIndex + (i + 1) * Count + j + 1);
		}
	}
	int PoleIndex = (int)Vertices.size() - 1;
	baseIndex = PoleIndex - Count;
	for (int i = 0; i < sliceCount; i++)
	{
		Indices.push_back(PoleIndex);
		Indices.push_back(baseIndex + i);
	}
}


void Capsule3::GetVertices(int stackCount, int sliceCount, std::vector<Vector3>& Vertices, std::vector<Vector3>* Normals)
{
	const float mPI = 2.0f * asinf(1.0f);

	float phiStep = mPI / stackCount;
	float thetaStep = 2.0f * mPI / sliceCount;
	float Length = (X1 - X0).Length();

	Vertices.push_back(Vector3(0, Length * 0.5f + Radius, 0));
	if (Normals) Normals->push_back(Vector3::UnitY());

	for (int i = 1; i < stackCount; i++)
	{
		float phi = i * phiStep;
		float height = i <= stackCount / 2 ? Length * 0.5f : -Length * 0.5f;
		for (int j = 0; j <= sliceCount; j++)
		{
			float theta = j * thetaStep;
			Vector3 p = Vector3(Radius * sinf(phi) * cosf(theta), height + Radius * cosf(phi), Radius * sinf(phi) * sinf(theta));
			Vertices.push_back(p);
			if (Normals) Normals->push_back(p);
		}
	}
	Vertices.push_back(Vector3(0, -Length * 0.5f - Radius, 0));
	if (Normals) Normals->push_back(-Vector3::UnitY());

	if (!IsYAxisAligned())
	{
		Matrix3 Rot;
		Rot.FromTwoAxis(Vector3::UnitY(), X1 - X0);

		Vector3* pV = Vertices.data();
		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			pV[i] = Rot * pV[i];
		}

		if (Normals)
		{
			Vector3* pN = Normals->data();
			for (size_t i = 0; i < Normals->size(); ++i)
			{
				pN[i] = Rot * pN[i];
			}
		}
	}

	Vector3 Center = GetCenter();
	if (Center.SquareLength() > 0.001f)
	{
		Vector3* pV = Vertices.data();
		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			pV[i] = pV[i] + Center;
		}
	}
}


bool Capsule3::intersectAxis(const Capsule3& capsule, const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& axis)
{
	float min0 = capsule.X0.Dot(axis);
	float max0 = capsule.X1.Dot(axis);
	if (min0 > max0)
		std::swap(min0, max0);

	const float MR = axis.Length() * capsule.Radius;
	min0 -= MR;
	max0 += MR;

	float min1 = std::min(A.Dot(axis), std::min(B.Dot(axis), C.Dot(axis)));
	float max1 = std::max(A.Dot(axis), std::max(B.Dot(axis), C.Dot(axis)));

	if (max0 < min1 || max1 < min0)
		return false;

	return true;
}

}
