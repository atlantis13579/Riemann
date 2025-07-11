#include <random>

#include "AxisAlignedBox3.h"
#include "Sphere3.h"
#include "Capsule3.h"
#include "Cylinder3.h"
#include "Segment3.h"
#include "Quad3.h"
#include "Triangle3.h"
#include "ConvexMesh.h"
#include "HeightField3.h"
#include "TriangleMesh.h"
#include "GJK.h"

namespace Riemann
{
static bool RayIntersectSphere(const Vector3& Origin, const Vector3& Direction, const Vector3& Center, float Radius, float* t)
{
	Vector3 oc = Origin - Center;
	float a = Direction.SquareLength();
	float b = 2.0f * oc.Dot(Direction);
	float c = oc.SquareLength() - Radius * Radius;
	float discriminant = b * b - 4 * a * c;
	if (discriminant < 0)
	{
		return false;
	}
	float h = (-b - sqrtf(discriminant)) / (2.0f * a);
	if (h >= 0)
	{
		*t = h;
		return true;
	}
	return false;
}

bool Sphere3::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	return RayIntersectSphere(Origin, Direction, Center, Radius, t);
}

bool Sphere3::IntersectPoint(const Vector3& Point) const
{
	float sqr_dist = (Point - Center).SquareLength();
	if (sqr_dist <= Radius * Radius)
	{
		return true;
	}
	return false;
}

// AABB intersects with a solid sphere or not by Jim Arvo, in "Graphics Gems":
bool Sphere3::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
{
	float dmin = 0.0f;
	for (int i = 0; i < 3; i++)
	{
		if (Center[i] < Bmin[i])
		{
			dmin += (Center[i] - Bmin[i]) * (Center[i] - Bmin[i]);
		}
		else if (Center[i] > Bmax[i])
		{
			dmin += (Center[i] - Bmax[i]) * (Center[i] - Bmax[i]);
		}
	}

	if (dmin <= Radius * Radius)
	{
		return true;
	}
	return false;
}

static bool SphereIntersectSphere(const Vector3& Center, float Radius, const Vector3& rCenter, float rRadius)
{
	float SqrDist = (Center - rCenter).SquareLength();
	return SqrDist <= (Radius + rRadius) * (Radius + rRadius);
}

bool Sphere3::IntersectSphere(const Vector3& _Center, float _Radius) const
{
	return SphereIntersectSphere(Center, Radius, _Center, _Radius);
}

bool Sphere3::IntersectCapsule(const Vector3& X0, const Vector3& X1, float rRadius) const
{
	float SqrDist = Segment3::SqrDistancePointToSegment(Center, X0, X1);
	return SqrDist <= (Radius + rRadius) * (Radius + rRadius);
}

static Vector3 ClosestPtPointTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c)
{
	// Check if P in vertex region outside A
	Vector3 ab = b - a;
	Vector3 ac = c - a;
	Vector3 ap = p - a;
	float d1 = DotProduct(ab, ap);
	float d2 = DotProduct(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f) return a; // barycentric coordinates (1,0,0)

	// Check if P in vertex region outside B
	Vector3 bp = p - b;
	float d3 = DotProduct(ab, bp);
	float d4 = DotProduct(ac, bp);
	if (d3 >= 0.0f && d4 <= d3) return b; // barycentric coordinates (0,1,0)

	// Check if P in edge region of AB, if so return projection of P onto AB
	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
		float v = d1 / (d1 - d3);
		return a + v * ab; // barycentric coordinates (1-v,v,0)
	}

	// Check if P in vertex region outside C
	Vector3 cp = p - c;
	float d5 = DotProduct(ab, cp);
	float d6 = DotProduct(ac, cp);
	if (d6 >= 0.0f && d5 <= d6) return c; // barycentric coordinates (0,0,1)

	// Check if P in edge region of AC, if so return projection of P onto AC
	float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
		float w = d2 / (d2 - d6);
		return a + w * ac; // barycentric coordinates (1-w,0,w)
	}

	// Check if P in edge region of BC, if so return projection of P onto BC
	float va = d3 * d6 - d5 * d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return b + w * (c - b); // barycentric coordinates (0,1-w,w)
	}

	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
}

bool Sphere3::IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
{
	float sqrDist = (A - Center).SquareLength();
	if (sqrDist <= Radius * Radius)
	{
		return true;
	}

	const Vector3 cp = ClosestPtPointTriangle(Center, A, B, C);
	sqrDist = (cp - Center).SquareLength();
	if (sqrDist <= Radius * Radius)
	{
		return true;
	}
	return false;
}

bool Sphere3::IntersectConvex(const ConvexMesh* convex) const
{
	return convex->IntersectSphere(Center, Radius);
}

bool Sphere3::IntersectHeightField(const HeightField3* hf) const
{
	return hf->IntersectSphere(Center, Radius);
}

bool Sphere3::IntersectTriangleMesh(const TriangleMesh* trimesh) const
{
	return trimesh->IntersectSphere(Center, Radius);
}

bool Sphere3::PenetrateSphere(const Vector3& rCenter, float rRadius, Vector3* Normal, float* Depth) const
{
	float SqrDist = (Center - rCenter).SquareLength();
	if (SqrDist > (Radius + rRadius) * (Radius + rRadius))
	{
		return false;
	}

	*Normal = Center - rCenter;
	float len = Normal->Length();
	if (len < 1e-6f)
	{
		*Normal = Vector3::UnitY();
	}
	else
	{
		*Normal = *Normal / len;
	}
	*Depth = Radius + rRadius - len;
	return true;
}

bool Sphere3::PenetratePlane(const Vector3& pNormal, float D, Vector3* Normal, float* Depth) const
{
	const float d = (Center + pNormal * D).Dot(pNormal);
	if (d > Radius)
	{
		return false;
	}

	*Normal = pNormal;
	*Depth = Radius - d;
	return true;
}

bool Sphere3::PenetrateCapsule(const Vector3& X0, const Vector3& X1, float rRadius, Vector3* Normal, float* Depth) const
{
	const float s = rRadius + Radius;
	Vector3 Closest = Segment3::ClosestPointOnSegmentToPoint(Center, X0, X1);
	if ((Center - Closest).SquareLength() > s * s)
	{
		return false;
	}

	*Normal = Center - Closest;

	const float len = Normal->Length();
	if (len < 1e-6f)
	{
		*Normal = Vector3::UnitY();
	}
	else
	{
		*Normal /= len;
	}
	*Depth = s - len;
	return true;
}

bool Sphere3::PenetrateOBB(const Vector3& rCenter, const Vector3& rExtent, const Matrix3& rRot, Vector3* Normal, float* Depth) const
{
	const Vector3 Delta = Center - rCenter;
	Vector3 localDelta = Delta * rRot;

	bool outside = false;

	if (localDelta.x < -rExtent.x)
	{
		outside = true;
		localDelta.x = -rExtent.x;
	}
	else if (localDelta.x > rExtent.x)
	{
		outside = true;
		localDelta.x = rExtent.x;
	}

	if (localDelta.y < -rExtent.y)
	{
		outside = true;
		localDelta.y = -rExtent.y;
	}
	else if (localDelta.y > rExtent.y)
	{
		outside = true;
		localDelta.y = rExtent.y;
	}

	if (localDelta.z < -rExtent.z)
	{
		outside = true;
		localDelta.z = -rExtent.z;
	}
	else if (localDelta.z > rExtent.z)
	{
		outside = true;
		localDelta.z = rExtent.z;
	}

	if (outside)
	{
		*Normal = Delta - rRot * localDelta;
		const float sqr = Normal->SquareLength();
		if (sqr > Radius * Radius)
			return false;
		const float len = sqrtf(sqr);

		*Depth = len - Radius;
		*Normal /= len;
		return true;
	}

	Vector3 localNormal;
	Vector3 d = rExtent - localDelta.Abs();

	if (d.y < d.x)
	{
		if (d.y < d.z)
		{
			localNormal = Vector3(0.0f, localDelta.y > 0.0f ? 1.0f : -1.0f, 0.0f);
			*Depth = -d.y;
		}
		else
		{
			localNormal = Vector3(0.0f, 0.0f, localDelta.z > 0.0f ? 1.0f : -1.0f);
			*Depth = -d.z;
		}
	}
	else
	{
		if (d.x < d.z)
		{
			localNormal = Vector3(localDelta.x > 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
			*Depth = -d.x;
		}
		else
		{
			localNormal = Vector3(0.0f, 0.0f, localDelta.z > 0.0f ? 1.0f : -1.0f);
			*Depth = -d.z;
		}
	}

	*Normal = rRot * localNormal;
	*Depth -= Radius;
	return true;
}

bool Sphere3::SweepAABB(const Vector3& Direction, const Vector3& bmin, const Vector3& bmax, Vector3* p, Vector3* n, float* t) const
{
	AxisAlignedBox3 box(bmin, bmax);
	GJKShapecast gjk;
    return gjk.Solve(Direction, this, &box, p, n, t);
}

bool Sphere3::SweepSphere(const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* p, Vector3* n, float* t) const
{
	Sphere3 s1(rCenter, rRadius + Radius);
	if (s1.IntersectRay(Center, Direction, t))
	{
		const Vector3 pt = Center + Direction * (*t);
		const Vector3 dir = pt - s1.Center;
		if (dir.SquareLength() > 1e-6f)
		{
			*n = dir.Unit();
		}
		else
		{
			*n = -Direction;
		}
		return true;
	}
	return false;
}

bool Sphere3::SweepPlane(const Vector3& Direction, const Vector3& Normal, float D, Vector3* p, Vector3* n, float* t) const
{
	float dist = Normal.Dot(Center) + D;
	if (fabsf(dist) <= Radius)
	{
		*t = 0.0f;
		*n = -Direction;
		return true;
	}
	else
	{
		const float denom = Normal.Dot(Direction);
		if (denom * dist >= 0.0f)
		{
			return false;
		}
		else
		{
			const float r = dist > 0.0f ? Radius : -Radius;
			*t = (r - dist) / denom;
			*n = Normal;
			return true;
		}
	}
}

bool Sphere3::SweepCylinder(const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* p, Vector3* n, float* t) const
{
    Cylinder3 cylinder(X0, X1, rRadius);
    GJKShapecast gjk;
    return gjk.Solve(Direction, this, &cylinder, p, n, t);
}

bool Sphere3::SweepCapsule(const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* p, Vector3* n, float* t) const
{
	Sphere3 sp2(Center, Radius);
	if (sp2.IntersectCapsule(X0, X1, rRadius))
	{
		*t = 0.0f;
		*n = -Direction;
		return true;
	}

	if (fabsf((X0 - X1).SquareLength()) < 1e-3f)
	{
        return sp2.SweepSphere(Direction, X0, rRadius, p, n, t);
	}

	Capsule3 capsule(X0, X1, Radius + rRadius);

	if (capsule.IntersectRay(Center, Direction, t))
	{
		if (*t >= 0.0f)
		{
			const Vector3 dir = *t * Direction;
			const Vector3 _X0 = X0 - dir;
			const Vector3 _X1 = X1 - dir;

			Vector3 closest = Segment3::ClosestPointOnSegmentToPoint(Center, _X0, _X1);

			*n = (closest - Center).Unit();
			return true;
		}
	}

	return false;
}

bool Sphere3::SweepConvex(const Vector3& Direction, const ConvexMesh* convex, Vector3* p, Vector3* n, float* t) const
{
	GJKShapecast gjk;
    return gjk.Solve(Direction, this, convex, p, n, t);
}

// Returns true if sphere can be tested against triangle vertex, false if edge test should be performed
static bool EdgeOrVertexTest(const Vector3& IntersectPoint, const Vector3* p, int v0, int v1, int v2, int *edge)
{
    const Vector3 edge0 = p[v0] - p[v1];
    const float edge0_length_sq = edge0.Dot(edge0);
    Vector3 diff = IntersectPoint - p[v1];
    if (edge0.Dot(diff) < edge0_length_sq)
    {
        *edge = v1;
        return false;
    }

    const Vector3 edge1 = p[v0] - p[v2];
    const float edge1_length_sq = edge1.Dot(edge1);
    diff = IntersectPoint - p[v2];
    if (edge1.Dot(diff) < edge1_length_sq)
    {
        *edge = v2;
        return false;
    }
    
    return true;
}

bool Sphere3::SweepTriangle(const Vector3& Direction, const Vector3 &A, const Vector3 &B, const Vector3 &C, Vector3* p, Vector3* n, float* t) const
{
    const Vector3 &center = Center;
    
    const bool testInitialOverlap = true;
    if (testInitialOverlap)
    {
        const Vector3 cp = ClosestPtPointTriangle(center, A, B, C);
        if ((cp - center).SquareLength() <= Radius * Radius)
        {
            *n = -Direction;
            *t = 0.0f;
            return true;
        }
    }
    
    const Vector3 Normal = Triangle3::CalculateNormal(A, B, C, false);
    Vector3 R = Normal * Radius;
    if (Direction.Dot(R) >= 0.0f)
        R = -R;

    float uu, vv, tt;
    const Vector3 BA = B - A;
    const Vector3 CA = C - A;
    const int r = Triangle3::RayIntersectTriangle2(center - R, Direction, A, BA, CA, &tt, &uu, &vv);
    if (!r)
        return false;
    if (r == 2)
    {
        if (tt < 0.0f)
            return false;
        *t = tt;
        return true;
    }

    // The triangle gets divided into the following areas (based on the barycentric coordinates (u,v)):
    //
    //               \   A0    /
    //                 \      /
    //                   \   /
    //                     \/ 0
    //            A02      *      A01
    //   u /              /   \          \ v
    //    *              /      \         *
    //                  /         \                        .
    //               2 /            \ 1
    //          ------*--------------*-------
    //               /                 \                .
    //        A2    /        A12         \   A1
    //
    //
    // Based on the area where the computed triangle plane intersection point lies in, a different sweep test will be applied.
    //
    // A) A01, A02, A12  : Test sphere against the corresponding edge
    // B) A0, A1, A2     : Test sphere against the corresponding vertex
    
    bool TestSphere;
    int e0,e1;
    Vector3 verts[3] = {A, B, C};
    if (uu < 0.0f)
    {
        if (vv < 0.0f)
        {
            // 0 or 0-1 or 0-2
            e0 = 0;
            const Vector3 intersectPoint = A * (1.0f - uu - vv) + B * uu + C * vv;
            TestSphere = EdgeOrVertexTest(intersectPoint, verts, 0, 1, 2, &e1);
        }
        else if (uu + vv > 1.0f)
        {
            // 2 or 2-0 or 2-1
            e0 = 2;
            const Vector3 intersectPoint = A * (1.0f - uu - vv) + B * uu + C * vv;
            TestSphere = EdgeOrVertexTest(intersectPoint, verts, 2, 0, 1, &e1);
        }
        else
        {
            // 0-2
            TestSphere = false;
            e0 = 0;
            e1 = 2;
        }
    }
    else
    {
        if (vv < 0.0f)
        {
            if (uu + vv > 1.0f)
            {
                // 1 or 1-0 or 1-2
                e0 = 1;
                const Vector3 intersectPoint = A * (1.0f - uu - vv) + B * uu + C * vv;
                TestSphere = EdgeOrVertexTest(intersectPoint, verts, 1, 0, 2, &e1);
            }
            else
            {
                // 0-1
                TestSphere = false;
                e0 = 0;
                e1 = 1;
            }
        }
        else
        {
            assert(uu + vv >= 1.0f);    // Else hit triangle
            // 1-2
            TestSphere = false;
            e0 = 1;
            e1 = 2;
        }
    }
    
    if (TestSphere)
    {
        Sphere3 sp(verts[e0], Radius);
        if (sp.IntersectRay(center, Direction, &tt))
        {
            *t = tt;
            return true;
        }
    }
    else
    {
        Capsule3 cap(verts[e0], verts[e1], Radius);
        if (cap.IntersectRay(center, Direction, &tt))
        {
            *t = tt;
            return true;
        }
    }

    return false;
}

bool Sphere3::SweepQuad(const Vector3& Direction, const Vector3 &A, const Vector3 &B, const Vector3 &C, const Vector3 &D, Vector3* p, Vector3* n, float* t) const
{
    //    Quad:
    //    A----C
    //    |   /|
    //    |  / |
    //    | /  |
    //    |/   |
    //    B----D

    const Vector3 BA = B - A;
    const Vector3 CA = C - A;
    const Vector3& center = Center;

    const float r2 = Radius * Radius;
    Vector3 Cp = Triangle3::ClosestPointOnTriangleToPoint(center, A, B, C);
    if ((Cp - center).SquareLength() <= r2)
    {
        *t = 0.0f;
        return true;
    }

    Cp = Triangle3::ClosestPointOnTriangleToPoint(center, D, C, B);
    if((Cp - center).SquareLength() <= r2)
    {
        *t = 0.0f;
        return true;
    }

    Vector3 normal = Triangle3::CalculateNormal(A, B, C, false);
    Vector3 R = normal * Radius;
    if (Direction.Dot(R) >= 0.0f)
        R = -R;

    // The first point of the sphere to hit the quad plane is the point of the sphere nearest to
    // the quad plane. Hence, we use center - (normal*radius) below.
    float tt, uu, vv;
    int r = Quad3::RayIntersectQuad2(center - R, Direction, A, BA, CA, &tt, &vv, &uu);
    if (r == 0)
    {
        return false;
    }
    if (r == 2)
    {
        if (tt < 0.0f)
            return false;
        *t = tt;
        return true;
    }

    bool TestSphere;
    int e0, e1;
    Vector3 quadVerts[] = {A, B, C, D};
    if (uu < 0.0f)
    {
        if (vv < 0.0f)
        {
            // 0 or 0-1 or 0-2
            e0 = 0;
            const Vector3 intersectPoint = A * (1.0f - uu - vv) + B * uu + C * vv;
            TestSphere = EdgeOrVertexTest(intersectPoint, quadVerts, 0, 1, 2, &e1);
        }
        else if(vv > 1.0f)
        {
            // 1 or 1-0 or 1-3
            e0 = 1;
            const Vector3 intersectPoint = A * (1.0f - uu - vv) + B * uu + C * vv;
            TestSphere = EdgeOrVertexTest(intersectPoint, quadVerts, 1, 0, 3, &e1);
        }
        else
        {
            // 0-1
            TestSphere = false;
            e0 = 0;
            e1 = 1;
        }
    }
    else if (uu > 1.0f)
    {
        if (vv < 0.0f)
        {
            // 2 or 2-0 or 2-3
            e0 = 2;
            const Vector3 intersectPoint = A * (1.0f - uu - vv) + B * uu + C * vv;
            TestSphere = EdgeOrVertexTest(intersectPoint, quadVerts, 2, 0, 3, &e1);
        }
        else if (vv > 1.0f)
        {
            // 3 or 3-1 or 3-2
            e0 = 3;
            const Vector3 intersectPoint = A * (1.0f - uu - vv) + B * uu + C * vv;
            TestSphere = EdgeOrVertexTest(intersectPoint, quadVerts, 3, 1, 2, &e1);
        }
        else
        {
            // 2-3
            TestSphere = false;
            e0 = 2;
            e1 = 3;
        }
    }
    else
    {
        if (vv < 0.0f)
        {
            // 0-2
            TestSphere = false;
            e0 = 0;
            e1 = 2;
        }
        else
        {
            assert(vv >= 1.0f);    // Else hit quad
            // 1-3
            TestSphere = false;
            e0 = 1;
            e1 = 3;
        }
    }
    
    if (TestSphere)
    {
        Sphere3 sp(quadVerts[e0], Radius);
        if (sp.IntersectRay(center, Direction, &tt))
        {
            *t = tt;
            return true;
        }
    }
    else
    {
        Capsule3 cap(quadVerts[e0], quadVerts[e1], Radius);
        if (cap.IntersectRay(center, Direction, &tt))
        {
            *t = tt;
            return true;
        }
    }
    
    return false;
}

bool Sphere3::SweepHeightField(const Vector3& Direction, const HeightField3* hf, Vector3* p, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Sphere3::SweepTriangleMesh(const Vector3& Direction, const TriangleMesh* trimesh, Vector3* p, Vector3* n, float* t) const
{
	// TODO
	return false;
}

static void MostSeparatedPointsOnAABB(const Vector3* points, int n, int& min, int& max)
{
	int minx = 0, maxx = 0, miny = 0, maxy = 0, minz = 0, maxz = 0;
	for (int i = 1; i < n; i++)
	{
		if (points[i].x < points[minx].x)
			minx = i;
		if (points[i].x > points[maxx].x)
			maxx = i;
		if (points[i].y < points[miny].y)
			miny = i;
		if (points[i].y > points[maxy].y)
			maxy = i;
		if (points[i].z < points[minz].z)
			minz = i;
		if (points[i].z > points[maxz].z)
			maxz = i;
	}

	float dist2x = DotProduct(points[maxx] - points[minx], points[maxx] - points[minx]);
	float dist2y = DotProduct(points[maxy] - points[miny], points[maxy] - points[miny]);
	float dist2z = DotProduct(points[maxz] - points[minz], points[maxz] - points[minz]);
	min = minx;
	max = maxx;
	if (dist2y > dist2x && dist2y > dist2z)
	{
		max = maxy;
		min = miny;
	}
	if (dist2z > dist2x && dist2z > dist2y)
	{
		max = maxz;
		min = minz;
	}
}

static Sphere3 SphereFromDistantPoints(const Vector3* points, int n)
{
	int min, max;
	MostSeparatedPointsOnAABB(points, n, min, max);

	Sphere3 sphere;
	sphere.Center = (points[min] + points[max]) * 0.5f;
	sphere.Radius = sqrtf(DotProduct(points[max] - sphere.Center, points[max] - sphere.Center));
	return sphere;
}

// static
Sphere3 Sphere3::ComputeBoundingSphere_MostSeparated(const Vector3* points, int n)
{
	Sphere3 sphere = SphereFromDistantPoints(points, n);
	for (int i = 0; i < n; ++i)
	{
		sphere.Encapsulate(points[i]);
	}
	return sphere;
}

// static
Sphere3 Sphere3::ComputeBoundingSphere_Eigen(const Vector3* points, int n)
{
	float eigens[3];
	Vector3 v[3];
	Matrix3 covariance_matrix = Matrix3::ComputeCovarianceMatrix(points, n);
	covariance_matrix.SolveEigenSymmetric(eigens, v);

	// Find the component with largest abs eigenvalue (largest axis)
	Vector3 dir = v[0];
	float maxe = fabsf(eigens[0]);
	if (fabsf(eigens[1]) > maxe)
	{
		dir = v[1];
		maxe = fabsf(eigens[1]);
	}
	if (fabsf(eigens[2]) > maxe)
	{
		dir = v[2];
		maxe = fabsf(eigens[2]);
	}

	int imin = -1, imax = -1;
	float minproj = FLT_MAX, maxproj = -FLT_MAX;
	for (int i = 0; i < n; i++)
	{
		float proj = points[i].Dot(dir);
		if (proj < minproj)
		{
			minproj = proj;
			imin = i;
		}
		if (proj > maxproj)
		{
			maxproj = proj;
			imax = i;
		}
	}
	Vector3 minpt = points[imin];
	Vector3 maxpt = points[imax];

	float dist = sqrtf(DotProduct(maxpt - minpt, maxpt - minpt));

	Sphere3 sphere;
	sphere.Radius = dist * 0.5f;
	sphere.Center = (minpt + maxpt) * 0.5f;

	for (int i = 0; i < n; ++i)
	{
		sphere.Encapsulate(points[i]);
	}
	return sphere;
}

// static
// Real Time Collision Detection - Christer Ericson
// Chapter 4.3.4, page 98-99
Sphere3 Sphere3::ComputeBoundingSphere_RitterIteration(const Vector3* _points, int n)
{
	std::vector<Vector3> points;
	points.resize(n);
	memcpy(points.data(), _points, n * sizeof(_points[0]));

	Sphere3 s = ComputeBoundingSphere_MostSeparated(points.data(), n);
	Sphere3 s2 = s;

	const int maxIterations = 8;
	for (int k = 0; k < maxIterations; k++)
	{
		s2.Radius = s2.Radius * 0.95f;

		for (int i = 0; i < n; i++)
		{
			int j = Maths::RandomInt(i + 1, n - 1);
			std::swap(points[i], points[j]);
			s2.Encapsulate(points[i]);
		}

		// found a tighter sphere
		if (s2.Radius < s.Radius)
			s = s2;
	}
	return s;
}

// Welzl, E. (1991). Smallest enclosing disks (balls and ellipsoids) (pp. 359-370). Springer Berlin Heidelberg.
static Sphere3 WelzlAlgorithm_Recursive(Vector3* points, int n, Vector3* support, int n_support)
{
	if (n == 0)
	{
		switch (n_support)
		{
		case 0: return Sphere3();
		case 1: return Sphere3(support[0]);
		case 2: return Sphere3(support[0], support[1]);
		case 3: return Sphere3(support[0], support[1], support[2]);
		case 4: return Sphere3(support[0], support[1], support[2], support[3]);
		}
	}

	int i = rand() % n;
	std::swap(points[i], points[n - 1]);

	Sphere3 min = WelzlAlgorithm_Recursive(points, n - 1, support, n_support);
	if ((points[n - 1] - min.Center).SquareLength() <= min.Radius * min.Radius)
	{
		return min;
	}
	support[n_support] = points[n - 1];
	return WelzlAlgorithm_Recursive(points, n - 1, support, n_support + 1);
}

// static
Sphere3 Sphere3::ComputeBoundingSphere_Welzl(const Vector3* _points, int n)
{
	std::vector<Vector3> points;
	points.resize(n);
	memcpy(points.data(), _points, n * sizeof(_points[0]));

	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle(points.begin(), points.end(), g);

	Vector3 support[4];
	Sphere3 s = WelzlAlgorithm_Recursive(points.data(), n, support, 0);
	s.Radius += kSphereEnlargeFactor;
	return s;
}


void Sphere3::GetVertices(int stackCount, int sliceCount, std::vector<Vector3>* Vertices, std::vector<Vector3>* Normals)
{
	const float mPI = 2.0f * asinf(1.0f);

	float phiStep = mPI / stackCount;
	float thetaStep = 2.0f * mPI / sliceCount;

	Vertices->push_back(Center + Vector3(0, Radius, 0));
	if (Normals) Normals->push_back(Vector3::UnitY());

	for (int i = 1; i < stackCount; i++)
	{
		float phi = i * phiStep;
		for (int j = 0; j <= sliceCount; j++)
		{
			float theta = j * thetaStep;
			Vector3 p = Vector3(sinf(phi) * cosf(theta), cosf(phi), sinf(phi) * sinf(theta)) * Radius;
			Vertices->push_back(Center + p);
			if (Normals) Normals->push_back(p);
		}
	}
	Vertices->push_back(Center + Vector3(0, -Radius, 0));
	if (Normals) Normals->push_back(-Vector3::UnitY());
}


void Sphere3::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
	const int stackCount = 8;
	const int sliceCount = 12;

	GetVertices(stackCount, sliceCount, &Vertices, &Normals);

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


void Sphere3::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
{
	const int stackCount = 6;
	const int sliceCount = 8;

	GetVertices(stackCount, sliceCount, &Vertices, nullptr);

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

}
