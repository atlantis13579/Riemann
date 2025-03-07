#include "Cylinder3.h"
#include "AxisAlignedBox3.h"
#include "Capsule3.h"
#include "Sphere3.h"
#include "Segment3.h"
#include "ConvexMesh.h"
#include "HeightField3.h"
#include "TriangleMesh.h"
#include "GJK.h"

namespace Riemann
{

bool Cylinder3::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	if (IntersectRayInfinityLength(Origin, Direction, Radius, t))
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
		if (Maths::SolveQuadratic(a, b, c) == 0)
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

bool Cylinder3::SweepAABB(const Vector3& Origin, const Vector3& Direction, const Vector3& bmin, const Vector3& bmax, Vector3* n, float* t) const
{
	AxisAlignedBox3 box(bmin, bmax);
	GJKShapecast gjk;
	return gjk.Solve(Origin, Direction, this, &box, n, t);
}

bool Cylinder3::SweepSphere(const Vector3& Origin, const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* n, float* t) const
{
	Sphere3 sp(rCenter, rRadius);
	GJKShapecast gjk;
	return gjk.Solve(Origin, Direction, this, &sp, n, t);
}

bool Cylinder3::SweepPlane(const Vector3& Origin, const Vector3& Direction, const Vector3& Normal, float D, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Cylinder3::SweepCapsule(const Vector3& Origin, const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* n, float* t) const
{
	Capsule3 capsule(X0, X1, rRadius);
	GJKShapecast gjk;
	return gjk.Solve(Origin, Direction, this, &capsule, n, t);
}

bool Cylinder3::SweepConvex(const Vector3& Origin, const Vector3& Direction, const ConvexMesh* convex, Vector3* n, float* t) const
{
	GJKShapecast gjk;
	return gjk.Solve(Origin, Direction, this, convex, n, t);
}

bool Cylinder3::SweepTriangle(const Vector3& Origin, const Vector3& Direction, const HeightField3* hf, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Cylinder3::SweepHeightField(const Vector3& Origin, const Vector3& Direction, const HeightField3* hf, Vector3* n, float* t) const
{
	// TODO
	return false;
}

bool Cylinder3::SweepTriangleMesh(const Vector3& Origin, const Vector3& Direction, const TriangleMesh* trimesh, Vector3* n, float* t) const
{
	// TODO
	return false;
}

int Cylinder3::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
{
	const Vector3* CylinderFaces = GetCylinderFaces();

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
	int nPts = sizeof(CylinderFaces) / sizeof(CylinderFaces[0]);
	for (int i = 0; i < nPts; ++i)
	{
		FacePoints[i] = Vector3(CylinderFaces[i].x * s.x, CylinderFaces[i].y * s.y, CylinderFaces[i].z * s.z);
	}
	return nPts;
}

void Cylinder3::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
	const Vector3* CylinderFaces = GetCylinderFaces();

	int nPts = sizeof(CylinderFaces) / sizeof(CylinderFaces[0]);
	Vector3 s = Vector3(Radius, Height * 0.5f, Radius);
	Vertices.resize(nPts * 2 + 2);
	Normals.resize(nPts * 2 + 2);

	Vertices[2 * nPts] = Vector3(0.0f, Height * 0.5f, 0.0f);
	Vertices[2 * nPts + 1] = Vector3(0.0f, -Height * 0.5f, 0.0f);
	Normals[2 * nPts] = Vector3::UnitY();
	Normals[2 * nPts + 1] = -Vector3::UnitY();
	for (int i = 0; i < nPts; ++i)
	{
		Vector3 p = Vector3(CylinderFaces[i].x * s.x, CylinderFaces[i].y * s.y, CylinderFaces[i].z * s.z);
		Vertices[i] = p;
		Vertices[i + nPts] = Vector3(p.x, -p.y, p.z);
		Normals[i] = (Vertices[2 * i] - Vertices[2 * nPts]).Unit();
		Normals[i + nPts] = (Vertices[2 * i + 1] - Vertices[2 * nPts + 1]).Unit();
	}

	Indices.resize(3 * 4 * nPts);
	for (int i = 0; i < nPts; ++i)
	{
		int j = (i + 1) % nPts;
		Indices[12 * i + 0] = j;
		Indices[12 * i + 1] = i;
		Indices[12 * i + 2] = i + nPts;

		Indices[12 * i + 3] = j + nPts;
		Indices[12 * i + 4] = j;
		Indices[12 * i + 5] = i + nPts;

		Indices[12 * i + 6] = i;
		Indices[12 * i + 7] = j;
		Indices[12 * i + 8] = 2 * nPts;

		Indices[12 * i + 9] = i + nPts;
		Indices[12 * i + 10] = j + nPts;
		Indices[12 * i + 11] = 2 * nPts + 1;
	}
}

void Cylinder3::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
{
	const Vector3* CylinderFaces = GetCylinderFaces();

	int nPts = sizeof(CylinderFaces) / sizeof(CylinderFaces[0]);
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