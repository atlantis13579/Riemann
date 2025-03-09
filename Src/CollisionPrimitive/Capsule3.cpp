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
	AxisAlignedBox3 box(bmin, bmax);
	GJKShapecast gjk;
	return gjk.Solve(Direction, this, &box, p, n, t);
}

bool Capsule3::SweepSphere(const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* p, Vector3* n, float* t) const
{
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
		if (plane.IntersectCapsule(X0 + Origin, X1 + Origin, Radius))
		{
			*n = -Direction;
			*t = 0.0f;
			return true;
		}
	}

	const Vector3 RelativeOrigin = Origin + GetSupport(Direction);
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

bool Capsule3::SweepConvex(const Vector3& Direction, const ConvexMesh* convex, Vector3* n, Vector3* p, float* t) const
{
	GJKShapecast gjk;
	return gjk.Solve(Direction, this, convex, p, n, t);
}

bool Capsule3::SweepTriangle(const Vector3& Direction, const Vector3 &A, const Vector3 &B, const Vector3 &C, Vector3* p, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Capsule3::SweepHeightField(const Vector3& Direction, const HeightField3* hf, Vector3* p, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Capsule3::SweepTriangleMesh(const Vector3& Direction, const TriangleMesh* trimesh, Vector3* p, Vector3* n, float* t) const
{
	// TODO
	return false;
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

	const float MR = axis.SquareLength() * capsule.Radius;
	min0 -= MR;
	max0 += MR;

	float min1 = std::min(A.Dot(axis), std::min(B.Dot(axis), C.Dot(axis)));
	float max1 = std::max(A.Dot(axis), std::max(B.Dot(axis), C.Dot(axis)));

	if (max0 < min1 || max1 < min0)
		return false;

	return true;
}

}
