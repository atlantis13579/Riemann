
#include "ConvexMesh.h"
#include "AxisAlignedBox3.h"
#include "Sphere3.h"
#include "Capsule3.h"
#include "GJK.h"
#include "../Geometry/ConvexHull3.h"

#include <algorithm>
#include <cstring>
#include <limits>

namespace Riemann
{
void ConvexMesh::SetConvexData(Vector3* verts, uint16_t nVerties,
	ConvexMeshFace* faces, uint16_t nFaces,
	uint16_t* edges, uint16_t nEdges,
	uint8_t* indices, uint16_t nIndices,
	bool shared_mem)
{

	NumVertices = nVerties;
	NumFaces = nFaces;
	NumEdges = nEdges;
	NumIndices = nIndices;

#ifndef USE_EDGE_DATA
	nEdges = 0;
#endif

	if (!shared_mem)
	{
		int buffer_size = sizeof(Vector3) * nVerties +
			sizeof(ConvexMeshFace) * nFaces +
			sizeof(ConvexMeshEdge) * nEdges +
			sizeof(uint8_t) * nIndices;

		Buffers.resize(buffer_size);

		Vertices = (Vector3*)Buffers.data();
		Faces = (ConvexMeshFace*)(Vertices + nVerties);
		Edges = (ConvexMeshEdge*)(Faces + nFaces);
		Indices = (uint8_t*)(Edges + nEdges);

		if (verts && nVerties)
		{
			std::memcpy(Vertices, verts, sizeof(Vector3) * nVerties);
		}
		if (faces && nFaces)
		{
			std::memcpy(Faces, faces, sizeof(ConvexMeshFace) * nFaces);
		}
		if (edges && nEdges)
		{
			std::memcpy(Edges, edges, sizeof(ConvexMeshEdge) * nEdges);
		}
		if (indices && nIndices)
		{
			std::memcpy(Indices, indices, sizeof(uint8_t) * nIndices);
		}
	}
	else
	{
		Vertices = (Vector3*)verts;
		Faces = faces;
		Edges = (ConvexMeshEdge*)edges;
		Indices = indices;
	}
	Bounds = ComputeBoundingVolume();
}

bool ConvexMesh::BuildFromPoints(const Vector3* points, int numPoints, const Vector3& localOrigin, float distanceTolerance)
{
	Release();
	if (points == nullptr || numPoints < 4)
	{
		return false;
	}

	std::vector<Vector3> localPoints;
	localPoints.reserve(numPoints);
	for (int pointIndex = 0; pointIndex < numPoints; ++pointIndex)
	{
		localPoints.push_back(points[pointIndex] - localOrigin);
	}

	ConvexHull3Options options;
	options.DistanceTolerance = distanceTolerance;

	ConvexHull3d hull;
	if (!BuildConvexHull3(localPoints, hull, options))
	{
		return false;
	}

	if (hull.verts.size() < 4 || hull.verts.size() > std::numeric_limits<uint8_t>::max() ||
		hull.faces.empty() || hull.faces.size() > std::numeric_limits<uint16_t>::max())
	{
		return false;
	}

	std::vector<ConvexMeshFace> faces;
	std::vector<uint8_t> indices;
	faces.reserve(hull.faces.size());
	indices.reserve(hull.faces.size() * 3);

	for (const HullFace3d& sourceFace : hull.faces)
	{
		if (sourceFace.verts.size() < 3 || sourceFace.verts.size() > std::numeric_limits<uint8_t>::max())
		{
			continue;
		}
		if (indices.size() + sourceFace.verts.size() > std::numeric_limits<uint16_t>::max())
		{
			return false;
		}

		const uint16_t first = (uint16_t)indices.size();
		for (short sourceIndex : sourceFace.verts)
		{
			if (sourceIndex < 0 || sourceIndex >= (short)hull.verts.size())
			{
				return false;
			}
			indices.push_back((uint8_t)sourceIndex);
		}

		faces.emplace_back(Plane3(sourceFace.norm, sourceFace.w), (uint8_t)sourceFace.verts.size(), first);
	}

	if (faces.size() < 4)
	{
		return false;
	}

	SetConvexData(
		hull.verts.data(), (uint16_t)hull.verts.size(),
		faces.data(), (uint16_t)faces.size(),
		nullptr, 0,
		indices.data(), (uint16_t)indices.size(),
		false);
	return NumVertices >= 4 && NumFaces >= 4;
}

bool ConvexMesh::ValidateStructure() const
{
	if (EulerNumber() != 2)
	{
		return false;
	}

	for (uint16_t i = 0; i < NumEdges; ++i)
	{
		if (Edges[i].s >= NumVertices)
		{
			return false;
		}
		if (Edges[i].e >= NumVertices)
		{
			return false;
		}
	}

	for (uint16_t i = 0; i < NumFaces; ++i)
	{
		for (uint8_t j = 0; j < Faces[i].numVerties; ++j)
		{
			if (Faces[i].first + j >= NumIndices)
			{
				return false;
			}
		}
	}

	for (uint16_t i = 0; i < NumIndices; ++i)
	{
		if (Indices[i] >= NumVertices)
		{
			return false;
		}
	}

	return true;
}

Box3 ConvexMesh::ComputeBoundingVolume() const
{
	Box3 box;
	box.SetEmpty();
	for (uint16_t i = 0; i < NumVertices; ++i)
	{
		box.Encapsulate(Vertices[i]);
	}
	return box;
}

bool ConvexMesh::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	int plane_test = 0;
	bool all_inside = true;
	float min_t = 0.0f, max_t = 1.0f;
	for (uint16_t i = 0; i < NumFaces; ++i)
	{
		const Plane3& p = Faces[i].plane;

		const float dist = Origin.Dot(p.Normal) + p.D;
		const bool is_outside = dist > 0.0f;
		if (is_outside)
		{
			all_inside = false;
		}

		float proj_dist = Direction.Dot(p.Normal);
		if (abs(proj_dist) >= 1e-6f)
		{
			float t_proj = -dist / proj_dist;
			if (proj_dist < 0.0f)
			{
				min_t = std::max(t_proj, min_t);
				plane_test |= 1;
			}
			else
			{
				max_t = std::min(t_proj, max_t);
				plane_test |= 2;
			}
		}
		else if (is_outside)
		{
			return false;
		}
	}

	if (plane_test == 3)
	{
		*t = min_t;
		return min_t <= max_t && max_t >= 0.0f;
	}

	return false;
}


template<class Shape>
static bool IntersectConvexMesh(const Shape *shape, const ConvexMesh *convex)
{
	GJKIntersection gjk;
	GJK_status gjk_status = gjk.Solve(shape, convex);
	if (gjk_status != GJK_status::Intersect)
	{
		return false;
	}
	return true;
}

bool ConvexMesh::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
{
	AxisAlignedBox3 box(Bmin, Bmax);
	return IntersectConvexMesh(&box, this);
}

bool ConvexMesh::IntersectSphere(const Vector3& iCenter, float iRadius) const
{
	Sphere3 sp(iCenter, iRadius);
	return IntersectConvexMesh(&sp, this);
}

bool ConvexMesh::IntersectCapsule(const Vector3& X0, const Vector3& X1, float rRadius) const
{
	Capsule3 capsule(X0, X1, rRadius);
	return IntersectConvexMesh(&capsule, this);
}

bool ConvexMesh::IntersectConvex(const ConvexMesh* convex) const
{
	return IntersectConvexMesh(convex, this);
}

bool ConvexMesh::CalculateVolumeProperties(MassParameters* p, float Density) const
{
	if (NumVertices == 0 || NumFaces == 0)
	{
		p->Volume = 0.0f;
		p->Mass = 0.0f;
		p->CenterOfMass = Vector3::Zero();
		p->BoundingVolume = Box3::Empty();
		p->InertiaMat = Matrix3::Identity();
		return true;
	}

	ConvexHull3d hull;
	hull.faces.resize(NumFaces);
	hull.verts.resize(NumVertices);

	for (uint16_t i = 0; i < NumVertices; ++i)
	{
		hull.verts[i] = Vertices[i];
	}
	for (uint16_t i = 0; i < NumFaces; ++i)
	{
		ConvexMeshFace& f = Faces[i];
		HullFace3d& face = hull.faces[i];
		face.norm = f.plane.Normal;
		face.w = f.plane.D;
		face.verts.resize(f.numVerties);
		for (uint8_t j = 0; j < f.numVerties; ++j)
		{
			face.verts[j] = Indices[f.first + j];
		}
	}

	float Mass, Volume;
	Vector3 CenterOfMass;
	p->InertiaMat = ComputePolyhedralInertiaTensor_VolumeIntegration(hull, Density, Volume, Mass, CenterOfMass);
	p->Mass = Mass;
	p->Volume = Volume;
	p->CenterOfMass = CenterOfMass;
	p->BoundingVolume = ComputeBoundingVolume();
	return true;
}

Vector3 ConvexMesh::GetSupport(const Vector3& Direction) const
{
	float max_dot = -FLT_MAX;
	Vector3 max_v = Vector3::Zero();

	for (uint16_t i = 0; i < NumVertices; ++i)
	{
		const float dot = Vertices[i].Dot(Direction);
		if (dot > max_dot)
		{
			max_dot = dot;
			max_v = Vertices[i];
		}
	}

	return max_v;
}

int ConvexMesh::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
{
	float max_dp = -FLT_MAX;
	uint16_t max_idx = 0xFFFF;
	for (uint16_t i = 0; i < NumFaces; ++i)
	{
		const Plane3& p = Faces[i].plane;
		const float dp = Direction.Dot(p.Normal);
		if (dp > max_dp)
		{
			max_dp = dp;
			max_idx = i;
		}
	}

	if (max_idx != 0xFFFF)
	{
		const ConvexMeshFace& hull = Faces[max_idx];
		for (uint8_t j = 0; j < hull.numVerties; ++j)
		{
			FacePoints[j] = Vertices[Indices[hull.first + j]];
		}
		return hull.numVerties;
	}

	return 0;
}

void ConvexMesh::GetMesh(std::vector<Vector3>& iVertices, std::vector<uint16_t>& iIndices, std::vector<Vector3>& _Normals)
{
	if (NumVertices < 3)
	{
		return;
	}

	iVertices.clear();
	iIndices.clear();
	_Normals.clear();

	for (uint16_t i = 0; i < NumFaces; ++i)
	{
		const ConvexMeshFace& face = Faces[i];
		if (face.numVerties < 3)
		{
			continue;
		}

		Vector3 normal = face.plane.Normal;
		if (normal.SafeNormalize() == 0.0f)
		{
			normal = Vector3::UnitY();
		}

		const uint16_t base = (uint16_t)iVertices.size();
		for (uint8_t j = 0; j < face.numVerties; ++j)
		{
			iVertices.push_back(Vertices[Indices[face.first + j]]);
			_Normals.push_back(normal);
		}

		for (uint8_t j = 1; j < face.numVerties - 1; ++j)
		{
			const uint16_t i0 = base;
			const uint16_t i1 = (uint16_t)(base + j);
			const uint16_t i2 = (uint16_t)(base + j + 1);
			const Vector3 triNormal = (iVertices[i1] - iVertices[i0]).Cross(iVertices[i2] - iVertices[i0]);
			iIndices.push_back(i0);
			if (triNormal.Dot(normal) >= 0.0f)
			{
				iIndices.push_back(i1);
				iIndices.push_back(i2);
			}
			else
			{
				iIndices.push_back(i2);
				iIndices.push_back(i1);
			}
		}
	}
}

void ConvexMesh::GetWireframe(std::vector<Vector3>& iVertices, std::vector<uint16_t>& iIndices)
{
	for (uint16_t i = 0; i < NumVertices; ++i)
	{
		iVertices.push_back(Vertices[i]);
	}
	for (uint16_t i = 0; i < NumEdges; ++i)
	{
		iIndices.push_back(Edges[i].s);
		iIndices.push_back(Edges[i].e);
	}
}
}
