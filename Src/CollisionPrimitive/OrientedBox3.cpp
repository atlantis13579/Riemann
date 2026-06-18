#include "OrientedBox3.h"
#include "AxisAlignedBox3.h"
#include "Capsule3.h"
#include "ConvexMesh.h"
#include "Cylinder3.h"
#include "GJK.h"
#include "HeightField3.h"
#include "Sphere3.h"
#include "Triangle3.h"
#include "TriangleMesh.h"

namespace Riemann
{

// static
OrientedBox3 OrientedBox3::ComputeBoundingOBB_PCA(const Vector3* points, int n)
{
	Vector3 CenterOfMass = Vector3::Zero();
	for (int i = 0; i < n; ++i)
	{
		CenterOfMass += points[i];
	}
	CenterOfMass *= (1.0f / n);

	const Matrix3 covariance_matrix = Matrix3::ComputeCovarianceMatrix(points, n);

	float eigens[3];
	Vector3 v[3];
	covariance_matrix.SolveEigenSymmetric(eigens, v);

	if (Determinant(v[0], v[1], v[2]) < 0)
	{
		v[2] = -v[2];
	}

	OrientedBox3 box;
	box.Center = CenterOfMass;
	box.Rotation = Matrix3(v[0].x, v[1].x, v[2].x,
		v[0].y, v[1].y, v[2].y,
		v[0].z, v[1].z, v[2].z);
	box.Extent = Vector3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	// TODO, optimize second / third axis using Polygon2D - MinAreaRect

	for (int i = 0; i < n; ++i)
	{
		Vector3 p = (box.Rotation * (points[i] - CenterOfMass)).Abs();
		box.Extent.x = std::max(box.Extent.x, p.x);
		box.Extent.y = std::max(box.Extent.y, p.y);
		box.Extent.z = std::max(box.Extent.z, p.z);
	}

	return box;
}

bool OrientedBox3::IntersectPoint(const Vector3& point) const
{
	AxisAlignedBox3 aabb(-Extent, Extent);
	return aabb.IntersectPoint((point - Center) * Rotation);	// inv(Rot) * v = transpose(Rot) * v = v^T * (Rot)
}

bool OrientedBox3::IntersectPlane(const Vector3& normal, const float D) const
{
	float r = Extent.x * fabsf(DotProduct(normal, Rotation.Column(0))) +
		Extent.y * fabsf(DotProduct(normal, Rotation.Column(1))) +
		Extent.z * fabsf(DotProduct(normal, Rotation.Column(2)));
	float s = normal.Dot(Center) + D;
	return fabsf(s) <= r;
}

bool OrientedBox3::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	AxisAlignedBox3 aabb(Center - Extent, Center + Extent);
	return aabb.IntersectRay(Origin * Rotation, Direction * Rotation, t);
}

bool OrientedBox3::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
{
	const OrientedBox3 obb((Bmax + Bmin) * 0.5f, (Bmax - Bmin) * 0.5f, Matrix3::Identity());
	return IntersectOBB(obb);
}

// Real Time Collision Detection - Christer Ericson
// Chapter 4.4.1, page 103-105
static bool OBBIntersectOBB(const Vector3& ca, const Vector3& ea, const Matrix3& ua, const Vector3& cb, const Vector3& eb, const Matrix3& ub)
{
	// Compute translation vector t
	Vector3 t = cb - ca;

	// Bring translation into a's coordinate frame
	t = Vector3(t.Dot(ua[0]), t.Dot(ua[1]), t.Dot(ua[2]));

	Matrix3 R, AbsR;
	float ra, rb;

	// Compute common subexpressions. Add in an epsilon term to
	// counteract arithmetic errors when two edges are parallel and
	// their cross product is (near) null (see text for details)
	const float EPLISON = 1e-6f;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			R[i][j] = ua[i].Dot(ub[j]);
			AbsR[i][j] = fabsf(R[i][j]) + EPLISON;
		}

	// Test axes L = A0, L = A1, L = A2
	for (int i = 0; i < 3; i++)
	{
		ra = ea[i];
		rb = eb[0] * AbsR[i][0] + eb[1] * AbsR[i][1] + eb[2] * AbsR[i][2];
		if (fabsf(t[i]) > ra + rb)		return false;
	}

	// Test axes L = B0, L = B1, L = B2
	for (int j = 0; j < 3; j++)
	{
		ra = ea[0] * AbsR[0][j] + ea[1] * AbsR[1][j] + ea[2] * AbsR[2][j];
		rb = eb[j];
		if (fabsf(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]) > ra + rb)	return false;
	}

	// Test Axis L = A0 x B0
	ra = ea[1] * AbsR[2][0] + ea[2] * AbsR[1][0];
	rb = eb[1] * AbsR[0][2] + eb[2] * AbsR[0][1];
	if (fabsf(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb)	return false;

	// Test Axis L = A0 x B1
	ra = ea[1] * AbsR[2][1] + ea[2] * AbsR[1][1];
	rb = eb[0] * AbsR[0][2] + eb[2] * AbsR[0][0];
	if (fabsf(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb)	return false;

	// Test Axis L = A0 x B2
	ra = ea[1] * AbsR[2][2] + ea[2] * AbsR[1][2];
	if (fabsf(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb)	return false;

	// Test Axis L = A1 x B0
	ra = ea[0] * AbsR[2][0] + ea[2] * AbsR[0][0];
	rb = eb[1] * AbsR[1][2] + eb[2] * AbsR[1][1];
	if (fabsf(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb)	return false;

	// Test Axis L = A1 x B1
	ra = ea[0] * AbsR[2][1] + ea[2] * AbsR[0][1];
	rb = eb[0] * AbsR[1][2] + eb[2] * AbsR[1][0];
	if (fabsf(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb)	return false;

	// Test Axis L = A1 x B2
	ra = ea[0] * AbsR[2][2] + ea[2] * AbsR[0][2];
	rb = eb[0] * AbsR[1][1] + eb[1] * AbsR[1][0];
	if (fabsf(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb)	return false;

	// Test Axis L = A2 x B0
	ra = ea[0] * AbsR[1][0] + ea[1] * AbsR[0][0];
	rb = eb[1] * AbsR[2][2] + eb[2] * AbsR[2][1];
	if (fabsf(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb)	return false;

	// Test Axis L = A2 x B1
	ra = ea[0] * AbsR[1][1] + ea[1] * AbsR[0][1];
	rb = eb[0] * AbsR[2][2] + eb[2] * AbsR[2][0];
	if (fabsf(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb)	return false;

	// Test Axis L = A2 x B2
	ra = ea[0] * AbsR[1][2] + ea[1] * AbsR[0][2];
	rb = eb[0] * AbsR[2][1] + eb[1] * AbsR[2][0];
	if (fabsf(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb)	return false;

	return true;
}

bool OrientedBox3::IntersectOBB(const OrientedBox3& obb) const
{
	return OBBIntersectOBB(Center, Extent, Rotation, obb.Center, obb.Extent, obb.Rotation);
}

bool OrientedBox3::IntersectOBB(const Vector3& iCenter, const Vector3& iExtent, const Matrix3& iRot) const
{
	return OBBIntersectOBB(Center, Extent, Rotation, iCenter, iExtent, iRot);
}

bool OrientedBox3::IntersectSphere(const Vector3& iCenter, float iRadius) const
{
	AxisAlignedBox3 aabb(Center - Extent, Center + Extent);
	return aabb.IntersectSphere(iCenter * Rotation, iRadius);	// inv(Rot) * v = transpose(Rot) * v = v^T * (Rot)
}

bool OrientedBox3::IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const
{
	AxisAlignedBox3 aabb(Center - Extent, Center + Extent);
	return aabb.IntersectCapsule(X0 * Rotation, X1 * Rotation, Radius);	// inv(Rot) * v = transpose(Rot) * v = v^T * (Rot)
}

bool OrientedBox3::IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
{
	AxisAlignedBox3 aabb(Center - Extent, Center + Extent);
	return aabb.IntersectTriangle(A * Rotation, B * Rotation, C * Rotation);	// inv(Rot) * v = transpose(Rot) * v = v^T * (Rot)
}

static bool testBoxBoxAxis(const OrientedBox3& obb0, const OrientedBox3& obb1, const Vector3& axis, Vector3* normal, float* depth)
{
	float min0, max0;
	obb0.ProjectAxis(axis, &min0, &max0);

	float min1, max1;
	obb1.ProjectAxis(axis, &min1, &max1);

	if (max0 < min1 || max1 < min0)
		return false;

	const float d = std::min(max0 - min1, max1 - min0);
	if (d < *depth)
	{
		*depth = d;
		*normal = axis;
	}
	return true;
}

bool OrientedBox3::PenetrateOBB(const OrientedBox3& obb, Vector3* Normal, float* Depth) const
{
	Vector3 n;
	float d = FLT_MAX;
	for (int i = 0; i < 3; ++i)
	{
		if (!testBoxBoxAxis(*this, obb, Rotation.Column(i), &n, &d))
			return false;
		if (!testBoxBoxAxis(*this, obb, obb.Rotation.Column(i), &n, &d))
			return false;
	}

	for (int j = 0; j < 3; ++j)
		for (int i = 0; i < 3; ++i)
		{
			Vector3 cross = Rotation.Row(i).Cross(obb.Rotation.Row(j));
			if (cross.SquareLength() > 1e-6f)
			{
				cross.Normalize();
				if (!testBoxBoxAxis(*this, obb, cross, &n, &d))
					return false;
			}
		}

	if (n.Dot(Center - obb.Center) < 0.0f)
		n = -n;

	*Normal = -n;
	*Depth = d;
	return true;
}

bool OrientedBox3::PenetrateOBB(const Vector3& iCenter, const Vector3& iExtent, const Matrix3& iRot, Vector3* Normal, float* Depth) const
{
	OrientedBox3 obb(iCenter, iExtent, iRot);
	return PenetrateOBB(obb, Normal, Depth);
}

bool OrientedBox3::PenetrateSphere(const Vector3& iCenter, float iRadius, Vector3* Normal, float* Depth) const
{
	Sphere3 sphere(iCenter, iRadius);
	if (sphere.PenetrateOBB(Center, Extent, Rotation, Normal, Depth))
	{
		*Normal = -*Normal;
		return true;
	}
	return false;
}

bool OrientedBox3::PenetratePlane(const Vector3& pNormal, float D, Vector3* Normal, float* Depth) const
{
	Vector3 v[8];
	GetVertices(v);

	float dmin = FLT_MAX;
	for (int i = 0; i < 8; ++i)
	{
		const float d = pNormal.Dot(v[i]) + D;
		if (d < dmin)
		{
			dmin = d;
		}
	}
	if (dmin > 0.0f)
	{
		return false;
	}

	*Normal = pNormal;
	*Depth = -dmin;
	return true;
}

static Vector3 OBBWorldToLocalPoint(const OrientedBox3& box, const Vector3& point)
{
	return (point - box.Center) * box.Rotation;
}

static Vector3 OBBWorldToLocalDirection(const OrientedBox3& box, const Vector3& direction)
{
	return direction * box.Rotation;
}

static Vector3 OBBLocalToWorldPoint(const OrientedBox3& box, const Vector3& point)
{
	return box.Center + box.Rotation * point;
}

static Vector3 OBBLocalToWorldDirection(const OrientedBox3& box, const Vector3& direction)
{
	return box.Rotation * direction;
}

static void TransformOBBSweepResultToWorld(const OrientedBox3& box, bool hit, Vector3* p, Vector3* n)
{
	if (!hit)
	{
		return;
	}
	if (p)
	{
		*p = OBBLocalToWorldPoint(box, *p);
	}
	if (n)
	{
		*n = OBBLocalToWorldDirection(box, *n);
		n->SafeNormalize();
	}
}

static Box3 ComputeOBBWorldBounds(const OrientedBox3& box)
{
	Vector3 vertices[8];
	box.GetVertices(vertices);

	Box3 bounds;
	bounds.SetEmpty();
	for (int i = 0; i < 8; ++i)
	{
		bounds.Encapsulate(vertices[i]);
	}
	return bounds;
}

static bool SweepMovingAABBAABBInterval(const Vector3& movingMin, const Vector3& movingMax, const Vector3& direction, const Vector3& staticMin, const Vector3& staticMax, float maxDist, float& enter, float& exit)
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

static bool SweepMovingAABBAABB(const Vector3& movingMin, const Vector3& movingMax, const Vector3& direction, const Vector3& staticMin, const Vector3& staticMax, float maxDist, float& toi)
{
	float enter, exit;
	if (!SweepMovingAABBAABBInterval(movingMin, movingMax, direction, staticMin, staticMax, maxDist, enter, exit))
	{
		return false;
	}
	toi = std::max(enter, 0.0f);
	return true;
}

static bool SetOBBInitialSweepHit(const OrientedBox3& box, const Vector3& Direction, Vector3* p, Vector3* n, float* t)
{
	if (t)
	{
		*t = 0.0f;
	}
	if (n)
	{
		*n = -Direction;
		n->SafeNormalize();
	}
	if (p)
	{
		*p = box.GetSupport(Direction);
	}
	return true;
}

Vector3 OrientedBox3::GetSupport(const Vector3& Direction) const
{
	const Vector3 localDir = OBBWorldToLocalDirection(*this, Direction);
	const Vector3 localSupport(
		localDir.x >= 0.0f ? Extent.x : -Extent.x,
		localDir.y >= 0.0f ? Extent.y : -Extent.y,
		localDir.z >= 0.0f ? Extent.z : -Extent.z);
	return OBBLocalToWorldPoint(*this, localSupport);
}

bool OrientedBox3::SweepAABB(const Vector3& Direction, const Vector3& bmin, const Vector3& bmax, Vector3* p, Vector3* n, float* t) const
{
	if (IntersectAABB(bmin, bmax))
	{
		return SetOBBInitialSweepHit(*this, Direction, p, n, t);
	}

	AxisAlignedBox3 box(bmin, bmax);
	GJKShapecast gjk;
	return gjk.Solve(Direction, this, &box, p, n, t);
}

bool OrientedBox3::SweepSphere(const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* p, Vector3* n, float* t) const
{
	AxisAlignedBox3 localBox(-Extent, Extent);
	if (localBox.IntersectSphere(OBBWorldToLocalPoint(*this, rCenter), rRadius))
	{
		return SetOBBInitialSweepHit(*this, Direction, p, n, t);
	}

	Sphere3 sp(rCenter, rRadius);
	GJKShapecast gjk;
	return gjk.Solve(Direction, this, &sp, p, n, t);
}

bool OrientedBox3::SweepPlane(const Vector3& Direction, const Vector3& Normal, float D, Vector3* p, Vector3* n, float* t) const
{
	AxisAlignedBox3 localBox(-Extent, Extent);
	const Vector3 localDir = OBBWorldToLocalDirection(*this, Direction);
	const Vector3 localNormal = OBBWorldToLocalDirection(*this, Normal);
	const float localD = Normal.Dot(Center) + D;

	Vector3 localPosition = Vector3::Zero();
	Vector3 localHitNormal = Vector3::Zero();
	float localT = 0.0f;
	const bool hit = localBox.SweepPlane(localDir, localNormal, localD, &localPosition, &localHitNormal, &localT);
	if (!hit)
	{
		return false;
	}

	if (t)
	{
		*t = localT;
	}
	if (n)
	{
		*n = OBBLocalToWorldDirection(*this, localHitNormal);
		n->SafeNormalize();
	}
	if (p)
	{
		const Vector3 support = localBox.GetSupport(localDir);
		*p = OBBLocalToWorldPoint(*this, support + localDir * localT);
	}
	return true;
}

bool OrientedBox3::SweepCylinder(const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* p, Vector3* n, float* t) const
{
	Cylinder3 cylinder(X0, X1, rRadius);
	GJKIntersection gjkIntersect;
	if (gjkIntersect.Solve(this, &cylinder) == GJK_status::Intersect)
	{
		return SetOBBInitialSweepHit(*this, Direction, p, n, t);
	}

	GJKShapecast gjk;
	return gjk.Solve(Direction, this, &cylinder, p, n, t);
}

bool OrientedBox3::SweepCapsule(const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* p, Vector3* n, float* t) const
{
	AxisAlignedBox3 localBox(-Extent, Extent);
	if (localBox.IntersectCapsule(OBBWorldToLocalPoint(*this, X0), OBBWorldToLocalPoint(*this, X1), rRadius))
	{
		return SetOBBInitialSweepHit(*this, Direction, p, n, t);
	}

	Capsule3 capsule(X0, X1, rRadius);
	GJKShapecast gjk;
	return gjk.Solve(Direction, this, &capsule, p, n, t);
}

bool OrientedBox3::SweepConvex(const Vector3& Direction, const ConvexMesh* convex, Vector3* p, Vector3* n, float* t) const
{
	if (convex != nullptr)
	{
		GJKIntersection gjkIntersect;
		if (gjkIntersect.Solve(this, convex) == GJK_status::Intersect)
		{
			return SetOBBInitialSweepHit(*this, Direction, p, n, t);
		}
	}

	GJKShapecast gjk;
	return gjk.Solve(Direction, this, convex, p, n, t);
}

bool OrientedBox3::SweepTriangle(const Vector3& Direction, const Vector3 &A, const Vector3 &B, const Vector3 &C, Vector3* p, Vector3* n, float* t) const
{
	AxisAlignedBox3 localBox(-Extent, Extent);
	const Vector3 localDir = OBBWorldToLocalDirection(*this, Direction);
	const Vector3 localA = OBBWorldToLocalPoint(*this, A);
	const Vector3 localB = OBBWorldToLocalPoint(*this, B);
	const Vector3 localC = OBBWorldToLocalPoint(*this, C);

	const bool hit = localBox.SweepTriangle(localDir, localA, localB, localC, p, n, t);
	TransformOBBSweepResultToWorld(*this, hit, p, n);
	return hit;
}

bool OrientedBox3::SweepHeightField(const Vector3& Direction, const HeightField3* hf, Vector3* p, Vector3* n, float* t) const
{
	if (hf == nullptr || hf->Cells == nullptr || hf->nX < 2 || hf->nZ < 2 || Direction.SquareLength() <= 1.0e-12f)
	{
		return false;
	}

	AxisAlignedBox3 localBox(-Extent, Extent);
	const Vector3 localDir = OBBWorldToLocalDirection(*this, Direction);
	const Vector3 localUnitDir = localDir.SafeUnit();
	const Box3 movingBounds = ComputeOBBWorldBounds(*this);

	float hfEnter, hfExit;
	if (!SweepMovingAABBAABBInterval(movingBounds.Min, movingBounds.Max, Direction, hf->BV.Min, hf->BV.Max, FLT_MAX, hfEnter, hfExit))
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
	Vector3 bestPosition = Vector3::Zero();
	Vector3 bestNormal = Vector3::Zero();

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
			if (!SweepMovingAABBAABB(movingBounds.Min, movingBounds.Max, Direction, cellBox.Min, cellBox.Max, bestDistance, cellToi))
			{
				continue;
			}

			Vector3 tris[6];
			const int numTriVerts = hf->GetCellTriangle(i, j, tris);
			for (int k = 0; k < numTriVerts; k += 3)
			{
				const Vector3 localA = OBBWorldToLocalPoint(*this, tris[k]);
				const Vector3 localB = OBBWorldToLocalPoint(*this, tris[k + 1]);
				const Vector3 localC = OBBWorldToLocalPoint(*this, tris[k + 2]);

				Vector3 localPosition, localNormal;
				float localT;
				if (!localBox.SweepTriangle(localDir, localA, localB, localC, &localPosition, &localNormal, &localT))
				{
					continue;
				}

				Vector3 triNormal = (localB - localA).Cross(localC - localA);
				if (triNormal.SafeNormalize() <= 1.0e-6f)
				{
					triNormal = -localUnitDir;
				}
				const float alignment = Triangle3::ComputeAlignmentValue(triNormal, localUnitDir);
				if (!hit || Triangle3::IsBetterTriangle(localT, alignment, bestDistance, bestAlignment))
				{
					hit = true;
					bestDistance = localT;
					bestAlignment = alignment;
					bestPosition = localPosition;
					bestNormal = localNormal;
				}
			}
		}
	}

	if (!hit)
	{
		return false;
	}

	if (t)
	{
		*t = bestDistance;
	}
	if (p)
	{
		*p = bestPosition;
	}
	if (n)
	{
		*n = bestNormal;
	}
	TransformOBBSweepResultToWorld(*this, true, p, n);
	return true;
}

bool OrientedBox3::SweepTriangleMesh(const Vector3& Direction, const TriangleMesh* trimesh, Vector3* p, Vector3* n, float* t) const
{
	if (trimesh == nullptr || trimesh->NumTriangles == 0 || Direction.SquareLength() <= 1.0e-12f)
	{
		return false;
	}

	AxisAlignedBox3 localBox(-Extent, Extent);
	const Vector3 localDir = OBBWorldToLocalDirection(*this, Direction);
	const Vector3 localUnitDir = localDir.SafeUnit();
	const Box3 movingBounds = ComputeOBBWorldBounds(*this);

	float meshToi;
	if (!SweepMovingAABBAABB(movingBounds.Min, movingBounds.Max, Direction, trimesh->BoundingVolume.Min, trimesh->BoundingVolume.Max, FLT_MAX, meshToi))
	{
		return false;
	}

	bool hit = false;
	float bestDistance = FLT_MAX;
	float bestAlignment = 2.0f;
	Vector3 bestPosition = Vector3::Zero();
	Vector3 bestNormal = Vector3::Zero();

	for (uint32_t i = 0; i < trimesh->NumTriangles; ++i)
	{
		const Vector3 worldA = trimesh->GetVertex(i, 0);
		const Vector3 worldB = trimesh->GetVertex(i, 1);
		const Vector3 worldC = trimesh->GetVertex(i, 2);
		const Box3 triBounds(worldA, worldB, worldC);

		float triToi;
		if (!SweepMovingAABBAABB(movingBounds.Min, movingBounds.Max, Direction, triBounds.Min, triBounds.Max, bestDistance, triToi))
		{
			continue;
		}

		const Vector3 localA = OBBWorldToLocalPoint(*this, worldA);
		const Vector3 localB = OBBWorldToLocalPoint(*this, worldB);
		const Vector3 localC = OBBWorldToLocalPoint(*this, worldC);

		Vector3 localPosition, localNormal;
		float localT;
		if (!localBox.SweepTriangle(localDir, localA, localB, localC, &localPosition, &localNormal, &localT))
		{
			continue;
		}

		Vector3 triNormal = (localB - localA).Cross(localC - localA);
		if (triNormal.SafeNormalize() <= 1.0e-6f)
		{
			triNormal = -localUnitDir;
		}
		const float alignment = Triangle3::ComputeAlignmentValue(triNormal, localUnitDir);
		if (!hit || Triangle3::IsBetterTriangle(localT, alignment, bestDistance, bestAlignment))
		{
			hit = true;
			bestDistance = localT;
			bestAlignment = alignment;
			bestPosition = localPosition;
			bestNormal = localNormal;
		}
	}

	if (!hit)
	{
		return false;
	}

	if (t)
	{
		*t = bestDistance;
	}
	if (p)
	{
		*p = bestPosition;
	}
	if (n)
	{
		*n = bestNormal;
	}
	TransformOBBSweepResultToWorld(*this, true, p, n);
	return true;
}

Vector3 OrientedBox3::ClosestPointToPoint(const Vector3& Point) const
{
	Vector3 d = Point - Center;
	Vector3 closestPt = Center;
	for (int i = 0; i < 3; i++)
	{
		float dist = d.Dot(Rotation.Column(i));
		if (dist > -Extent[i])
			dist = -Extent[i];
		if (dist < -Extent[i])
			dist = -Extent[i];
		closestPt += dist * Rotation.Column(i);
	}
	return closestPt;
}

float OrientedBox3::SqrDistanceToLine(const Vector3& P0, const Vector3& Direction, float* t) const
{
	AxisAlignedBox3 aabb(Center - Extent, Center + Extent);
	float SqrDist = aabb.SqrDistanceToLine(P0 * Rotation, Direction * Rotation, t);
	return SqrDist;
}

float OrientedBox3::SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const
{
	AxisAlignedBox3 aabb(Center - Extent, Center + Extent);
	float SqrDist = aabb.SqrDistanceToSegment(P0 * Rotation, P1 * Rotation);
	return SqrDist;
}

float OrientedBox3::SqrDistanceToPoint(const Vector3& Point) const
{
	Vector3 closestPt = ClosestPointToPoint(Point);
	float SqrDist = closestPt.SquareLength();
	return SqrDist;
}

}
