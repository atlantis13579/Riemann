
#include "ConvexMesh.h"
#include "AxisAlignedBox3.h"
#include "Sphere3.h"
#include "Capsule3.h"
#include "GJK.h"
#include "../Geometry/ConvexHull3.h"

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

		float dist = Origin.Dot(p.Normal) + p.D;
		bool is_outside = dist > 0.0f;
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
	MinkowskiSumTwoShape<Shape, ConvexMesh> m(shape, convex);
	GJKIntersection gjk;
	GJK_status gjk_status = gjk.Solve(&m);
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

bool ConvexMesh::IntersectSphere(const Vector3& _Center, float _Radius) const
{
	Sphere3 sp(_Center, _Radius);
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

void ConvexMesh::GetMesh(std::vector<Vector3>& _Vertices, std::vector<uint16_t>& _Indices, std::vector<Vector3>& _Normals)
{
	if (NumVertices < 3)
	{
		return;
	}

	Vector3 Center = Vector3::Zero();

	for (uint16_t i = 0; i < NumVertices; ++i)
	{
		Center += Vertices[i];
	}
	Center = Center / NumVertices;

	for (uint16_t i = 0; i < NumVertices; ++i)
	{
		_Vertices.push_back(Vertices[i]);
		_Normals.push_back((Vertices[i] - Center).Unit());
	}

	for (uint16_t i = 0; i < NumFaces; ++i)
	{
		const ConvexMeshFace& face = Faces[i];
		for (uint8_t j = 1; j < face.numVerties - 1; ++j)
		{
			_Indices.push_back(Indices[face.first]);
			_Indices.push_back(Indices[face.first + j]);
			_Indices.push_back(Indices[face.first + j + 1]);
		}
	}
}

void ConvexMesh::GetWireframe(std::vector<Vector3>& _Vertices, std::vector<uint16_t>& _Indices)
{
	for (uint16_t i = 0; i < NumVertices; ++i)
	{
		_Vertices.push_back(Vertices[i]);
	}
	for (uint16_t i = 0; i < NumEdges; ++i)
	{
		_Indices.push_back(Edges[i].s);
		_Indices.push_back(Edges[i].e);
	}
}
}