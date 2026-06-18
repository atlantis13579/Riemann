#include "Cylinder3.h"
#include "AxisAlignedBox3.h"
#include "Capsule3.h"
#include "Sphere3.h"
#include "Segment3.h"
#include "ConvexMesh.h"
#include "HeightField3.h"
#include "Triangle3.h"
#include "TriangleMesh.h"
#include "GJK.h"

namespace Riemann
{

bool Cylinder3::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	if (!IntersectRayInfinityLength(Origin, Direction, Radius, t))
	{
		return false;
	}

	if (fabsf(Origin.y + (*t) * Direction.y) <= Height * 0.5f)
	{
		return true;
	}

	if (fabsf(Direction.y) > 1e-6f)
	{
		float h = Direction.y < 0.0f ? (Height * 0.5f - Origin.y) / Direction.y : -(Height * 0.5f + Origin.y) / Direction.y;

		if (h >= 0.0f)
		{
			Vector3 pos = Origin + h * Direction;
			float sq = pos.x * pos.x + pos.z * pos.z;
			if (sq <= Radius * Radius)
			{
				*t = h;
				return true;
			}
		}
	}

	return false;
}

bool Cylinder3::IntersectRayInfinityLength(const Vector3& Origin, const Vector3& Direction, float Radius, float* t)
{
	Vector3 OriginXZ = Vector3(Origin.x, 0.0f, Origin.z);
	float sqr = OriginXZ.SquareLength();
	if (sqr > Radius * Radius)
	{
		Vector3 DirectionXZ = Vector3(Direction.x, 0.0f, Direction.z);
		float a = DirectionXZ.SquareLength();
		float b = 2.0f * OriginXZ.Dot(DirectionXZ);
		float c = sqr - Radius * Radius;

		float roots[2];
		if (Maths::SolveQuadratic(a, b, c, roots) == 0)
		{
			return false;
		}
		float h = std::min(roots[0], roots[1]);
		if (h >= 0.0f)
		{
			*t = h;
			return true;
		}
	}
	*t = 0.0f;
	return true;
}

bool Cylinder3::IntersectSegment(const Vector3& P0, const Vector3& P1) const
{
	Vector3 sa, sb;
	Vector3 d = P0 - P1, m = sa - P1, n = sb - sa;
	float md = d.Dot(d);
	float nd = d.Dot(d);
	float dd = d.Dot(d);
	if (md < 0.0f && md + nd < 0.0f)
		return false;
	if (md > dd && md + nd > dd)
		return false;
	float nn = n.Dot(n);
	float mn = n.Dot(m);
	float a = dd * nn - nd * nd;
	float k = m.Dot(m) - Radius * Radius;
	float c = dd * k - md * md;
	if (fabsf(a) < 1e-6f)
	{
		return c <= 0;
	}
	float b = dd * mn - nd * md;
	float discr = b * b - a * c;
	if (discr < 0)
		return false;
	float t = (-b - sqrtf(discr)) / a;
	if (t < 0.0f || t > 1.0f)
		return false;
	if (md + t * nd < 0.0f)
	{
		if (nd <= 0.0f)
			return false;
		t = -md / nd;
		return k + 2 * t * (mn + t * nn) <= 0.0f;
	}
	else if (md + t * nd > dd)
	{
		if (nd >= 0.0f)
			return false;
		t = (dd - md) / nd;
		return k + dd - 2 * md + t * (2 * (mn - nd) + t * nn) <= 0.0f;
	}
	return true;
}

bool Cylinder3::SweepAABB(const Vector3& Direction, const Vector3& bmin, const Vector3& bmax, Vector3* p, Vector3* n, float* t) const
{
	AxisAlignedBox3 box(bmin, bmax);
	GJKIntersection gjkIntersect;
	if (gjkIntersect.Solve(this, &box) == GJK_status::Intersect)
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
	return gjk.Solve(Direction, this, &box, p, n, t);
}

bool Cylinder3::SweepSphere(const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* p, Vector3* n, float* t) const
{
	Sphere3 sp(rCenter, rRadius);
	GJKIntersection gjkIntersect;
	if (gjkIntersect.Solve(this, &sp) == GJK_status::Intersect)
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
	return gjk.Solve(Direction, this, &sp, p, n, t);
}

bool Cylinder3::SweepPlane(const Vector3& Direction, const Vector3& Normal, float D, Vector3* p, Vector3* n, float* t) const
{
	Plane3 plane(Normal, D);

    const Vector3 Origin = GetCenter();
	const float dp = Direction.Dot(Normal);
	if (fabsf(dp) < 1e-6f)
	{
        if (plane.IntersectCylinder(X0, X1, Radius))
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

bool Cylinder3::SweepCylinder(const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* p, Vector3* n, float* t) const
{
    Cylinder3 cylinder(X0, X1, rRadius);
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

bool Cylinder3::SweepCapsule(const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* p, Vector3* n, float* t) const
{
	Capsule3 capsule(X0, X1, rRadius);
	GJKIntersection gjkIntersect;
	if (gjkIntersect.Solve(this, &capsule) == GJK_status::Intersect)
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
	return gjk.Solve(Direction, this, &capsule, p, n, t);
}

bool Cylinder3::SweepConvex(const Vector3& Direction, const ConvexMesh* convex, Vector3* p, Vector3* n, float* t) const
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

struct CylinderSweepTriangleShape
{
	Vector3 A, B, C;

	Vector3 GetCenter() const
	{
		return (A + B + C) * (1.0f / 3.0f);
	}

	Vector3 GetSupport(const Vector3& Direction) const
	{
		const float da = A.Dot(Direction);
		const float db = B.Dot(Direction);
		const float dc = C.Dot(Direction);
		if (da >= db && da >= dc)
		{
			return A;
		}
		return db >= dc ? B : C;
	}
};

static bool CylinderSweepMovingAABBAABBInterval(const Vector3& movingMin, const Vector3& movingMax, const Vector3& direction, const Vector3& staticMin, const Vector3& staticMax, float maxDist, float& enter, float& exit)
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

static bool CylinderSweepMovingAABBAABB(const Vector3& movingMin, const Vector3& movingMax, const Vector3& direction, const Vector3& staticMin, const Vector3& staticMax, float maxDist, float& toi)
{
	float enter, exit;
	if (!CylinderSweepMovingAABBAABBInterval(movingMin, movingMax, direction, staticMin, staticMax, maxDist, enter, exit))
	{
		return false;
	}
	toi = std::max(enter, 0.0f);
	return true;
}

static void ComputeCylinderTriangleSweepResult(const Cylinder3& cylinder, const Vector3& direction, float hitTime, const Vector3& A, const Vector3& B, const Vector3& C, Vector3* p, Vector3* n, float* t)
{
	if (t)
	{
		*t = hitTime;
	}

	const Vector3 movedCenter = cylinder.GetCenter() + direction * hitTime;
	const Vector3 localHit = Triangle3::ClosestPointOnTriangleToPoint(movedCenter, A, B, C);
	if (p)
	{
		*p = localHit;
	}
	if (n)
	{
		*n = movedCenter - localHit;
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

static void TestCylinderTriangleForBestSweep(const Cylinder3& cylinder, const Vector3& direction, const Vector3& unitDir, const Vector3& A, const Vector3& B, const Vector3& C, float& bestDistance, float& bestAlignment, Vector3 bestTri[3], bool& hit)
{
	Vector3 unusedP, unusedN;
	float currentDistance;
	if (!cylinder.SweepTriangle(direction, A, B, C, &unusedP, &unusedN, &currentDistance))
	{
		return;
	}
	if (currentDistance > bestDistance)
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

bool Cylinder3::SweepTriangle(const Vector3& Direction, const Vector3 &A, const Vector3 &B, const Vector3 &C, Vector3* p, Vector3* n, float* t) const
{
	if (Direction.SquareLength() <= 1.0e-12f)
	{
		return false;
	}

	CylinderSweepTriangleShape tri = { A, B, C };
	GJKIntersection gjkIntersect;
	if (gjkIntersect.Solve(this, &tri) == GJK_status::Intersect)
	{
		ComputeCylinderTriangleSweepResult(*this, Direction, 0.0f, A, B, C, p, n, t);
		return true;
	}

	const Box3 bounds = CalculateBoundingVolume();
	AxisAlignedBox3 boundBox(bounds.Min, bounds.Max);
	if (boundBox.SweepTriangle(Direction, A, B, C, p, n, t))
	{
		return true;
	}

	GJKShapecast gjk;
	Vector3 gjkP, gjkN;
	float hitTime;
	if (!gjk.Solve(Direction, this, &tri, &gjkP, &gjkN, &hitTime))
	{
		return false;
	}

	ComputeCylinderTriangleSweepResult(*this, Direction, hitTime, A, B, C, p, n, t);
	return true;
}

bool Cylinder3::SweepHeightField(const Vector3& Direction, const HeightField3* hf, Vector3* p, Vector3* n, float* t) const
{
	if (hf == nullptr || hf->Cells == nullptr || hf->nX < 2 || hf->nZ < 2 || Direction.SquareLength() <= 1.0e-12f)
	{
		return false;
	}

	const Box3 movingBounds = CalculateBoundingVolume();
	float hfEnter, hfExit;
	if (!CylinderSweepMovingAABBAABBInterval(movingBounds.Min, movingBounds.Max, Direction, hf->BV.Min, hf->BV.Max, FLT_MAX, hfEnter, hfExit))
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
			if (!CylinderSweepMovingAABBAABB(movingBounds.Min, movingBounds.Max, Direction, cellBox.Min, cellBox.Max, bestDistance, cellToi))
			{
				continue;
			}

			Vector3 tris[6];
			const int numTriVerts = hf->GetCellTriangle(i, j, tris);
			for (int k = 0; k < numTriVerts; k += 3)
			{
				TestCylinderTriangleForBestSweep(*this, Direction, unitDir, tris[k], tris[k + 1], tris[k + 2], bestDistance, bestAlignment, bestTri, hit);
			}
		}
	}

	if (!hit)
	{
		return false;
	}

	ComputeCylinderTriangleSweepResult(*this, Direction, bestDistance, bestTri[0], bestTri[1], bestTri[2], p, n, t);
	return true;
}

bool Cylinder3::SweepTriangleMesh(const Vector3& Direction, const TriangleMesh* trimesh, Vector3* p, Vector3* n, float* t) const
{
	if (trimesh == nullptr || trimesh->NumTriangles == 0 || Direction.SquareLength() <= 1.0e-12f)
	{
		return false;
	}

	const Box3 movingBounds = CalculateBoundingVolume();
	float meshToi;
	if (!CylinderSweepMovingAABBAABB(movingBounds.Min, movingBounds.Max, Direction, trimesh->BoundingVolume.Min, trimesh->BoundingVolume.Max, FLT_MAX, meshToi))
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
		if (!CylinderSweepMovingAABBAABB(movingBounds.Min, movingBounds.Max, Direction, triBounds.Min, triBounds.Max, bestDistance, triToi))
		{
			continue;
		}

		TestCylinderTriangleForBestSweep(*this, Direction, unitDir, A, B, C, bestDistance, bestAlignment, bestTri, hit);
	}

	if (!hit)
	{
		return false;
	}

	ComputeCylinderTriangleSweepResult(*this, Direction, bestDistance, bestTri[0], bestTri[1], bestTri[2], p, n, t);
	return true;
}

int Cylinder3::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
{
	const auto& CylinderFaces = GetCylinderFaces();

	const float HalfHeight = Height * 0.5f;
	const float o = sqrtf(Direction.x * Direction.x + Direction.z * Direction.z);

	// side
	if (o * HalfHeight > Radius * fabsf(Direction.y))
	{
		FacePoints[0] = Vector3(-Direction.x * Radius / o, HalfHeight, -Direction.z * Radius / o);
		FacePoints[1] = Vector3(-Direction.x * Radius / o, -HalfHeight, -Direction.z * Radius / o);
		return 2;
	}

	// top or bottom
	const float sy = Direction.y > 0 ? 1.0f : -1.0f;
	Vector3 s(Radius, HalfHeight * sy, Radius);
	for (size_t i = 0; i < CylinderFaces.size(); ++i)
	{
		FacePoints[i] = Vector3(CylinderFaces[i].x * s.x, CylinderFaces[i].y * s.y, CylinderFaces[i].z * s.z);
	}
	return (int)CylinderFaces.size();
}

void Cylinder3::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
    const auto& CylinderFaces = GetCylinderFaces();

	int nPts = (int)CylinderFaces.size();
	Vector3 s = Vector3(Radius, Height * 0.5f, Radius);
	const int sideTop = 0;
	const int sideBottom = sideTop + nPts;
	const int topCap = sideBottom + nPts;
	const int bottomCap = topCap + nPts;
	const int topCenter = bottomCap + nPts;
	const int bottomCenter = topCenter + 1;
	Vertices.resize(nPts * 4 + 2);
	Normals.resize(nPts * 4 + 2);

	Vertices[topCenter] = Vector3(0.0f, Height * 0.5f, 0.0f);
	Vertices[bottomCenter] = Vector3(0.0f, -Height * 0.5f, 0.0f);
	Normals[topCenter] = Vector3::UnitY();
	Normals[bottomCenter] = -Vector3::UnitY();
	for (int i = 0; i < nPts; ++i)
	{
		Vector3 p = Vector3(CylinderFaces[i].x * s.x, CylinderFaces[i].y * s.y, CylinderFaces[i].z * s.z);
		Vector3 top = p;
		Vector3 bottom = Vector3(p.x, -p.y, p.z);
		Vector3 sideNormal = Vector3(p.x, 0.0f, p.z);
		if (sideNormal.SafeNormalize() == 0.0f)
		{
			sideNormal = Vector3::UnitX();
		}

		Vertices[sideTop + i] = top;
		Vertices[sideBottom + i] = bottom;
		Normals[sideTop + i] = sideNormal;
		Normals[sideBottom + i] = sideNormal;

		Vertices[topCap + i] = top;
		Vertices[bottomCap + i] = bottom;
		Normals[topCap + i] = Vector3::UnitY();
		Normals[bottomCap + i] = -Vector3::UnitY();
	}

	Indices.resize(3 * 4 * nPts);
	const auto u16 = [](int value) { return static_cast<uint16_t>(value); };
	for (int i = 0; i < nPts; ++i)
	{
		int j = (i + 1) % nPts;
		Indices[12 * i + 0] = u16(sideTop + j);
		Indices[12 * i + 1] = u16(sideTop + i);
		Indices[12 * i + 2] = u16(sideBottom + i);

		Indices[12 * i + 3] = u16(sideBottom + j);
		Indices[12 * i + 4] = u16(sideTop + j);
		Indices[12 * i + 5] = u16(sideBottom + i);

		Indices[12 * i + 6] = u16(topCap + i);
		Indices[12 * i + 7] = u16(topCap + j);
		Indices[12 * i + 8] = u16(topCenter);

		Indices[12 * i + 9] = u16(bottomCap + j);
		Indices[12 * i + 10] = u16(bottomCap + i);
		Indices[12 * i + 11] = u16(bottomCenter);
	}
}

void Cylinder3::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
{
    const auto& CylinderFaces = GetCylinderFaces();

	int nPts = (int)CylinderFaces.size();
	Vector3 s = Vector3(Radius, Height * 0.5f, Radius);
	Vertices.resize(nPts * 2 + 2);
	Vertices[2 * nPts] = Vector3(0.0f, Height * 0.5f, 0.0f);
	Vertices[2 * nPts + 1] = Vector3(0.0f, -Height * 0.5f, 0.0f);
	for (int i = 0; i < nPts; ++i)
	{
		Vector3 p = Vector3(CylinderFaces[i].x * s.x, CylinderFaces[i].y * s.y, CylinderFaces[i].z * s.z);
		Vertices[i] = p;
		Vertices[i + nPts] = Vector3(p.x, -p.y, p.z);
	}

	Indices.resize(2 * 3 * nPts);
	for (int i = 0; i < nPts; ++i)
	{
		int j = (i + 1) % nPts;
		Indices[6 * i + 0] = j;
		Indices[6 * i + 1] = i;

		Indices[6 * i + 2] = j + nPts;
		Indices[6 * i + 3] = i + nPts;

		Indices[6 * i + 4] = i;
		Indices[6 * i + 5] = i + nPts;
	}
}

}
