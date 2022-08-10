
#pragma once

#include <stdint.h>
#include <vector>
#include "ShapeType.h"
#include "Plane3d.h"
#include "../Maths/Vector3.h"
#include "../Maths/Box3d.h"
#include "../Maths/Matrix3.h"

struct HullVertex3d
{
	explicit HullVertex3d(const Vector3& _p)
	{
		p = _p;
	}
	Vector3			p;
};

struct HullFace3d
{
	explicit HullFace3d(const Plane3d& pl, uint8_t nVerties, uint16_t first_idx)
	{
		plane = pl;
		numVerties = nVerties;
		first = first_idx;
	}
	Plane3d			plane;
	uint8_t			numVerties;
	uint16_t		first;
};

struct HullEdge3d
{
	explicit HullEdge3d(uint16_t _s, uint16_t _e)
	{
		s = _s;
		e = _e;
	}
	uint16_t		s;
	uint16_t		e;
};

static_assert(sizeof(HullVertex3d) == 12, "sizeof(HullVertex3d) not right");

class ConvexMesh
{
public:
	Vector3						CenterOfMass;
	Box3d						BoundingVolume;
	Matrix3						Inertia;
	std::vector<HullVertex3d>	Vertices;
	std::vector<HullEdge3d>		Edges;
	std::vector<HullFace3d>		Faces;
	std::vector<uint8_t>		Indices;
	uint16_t					NumVertices;
	uint16_t					NumEdges;
	uint16_t					NumFaces;
	uint16_t					NumIndices;

	ConvexMesh()
	{
		Release();
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::CONVEX_MESH;
	}

	void		Release()
	{
		NumVertices = NumFaces = NumEdges = 0;
		Vertices.clear();
		Edges.clear();
		Faces.clear();
		Indices.clear();
	}

	int			EulerNumber() const
	{
		return NumVertices - NumEdges + NumFaces;
	}

	void		AddFace(const Plane3d& p, uint8_t nVerties, uint16_t idx)
	{
		Faces.emplace_back(p, nVerties, idx);
		NumFaces++;
	}

	void		SetVerties(const Vector3* Verts, uint16_t nVerties)
	{
		NumVertices = nVerties;
		for (uint32_t i = 0; i < nVerties; ++i)
		{
			Vertices.emplace_back(Verts[i]);
		}
	}

	void		SetEdges(const uint16_t* edges, uint16_t nEdges)
	{
		NumEdges = nEdges;
		for (uint16_t i = 0; i < nEdges; ++i)
		{
			Edges.emplace_back(edges[2 * i], edges[2 * i + 1]);
		}
	}

	void		SetIndices(const uint8_t* indices, uint16_t nIndices)
	{
		NumIndices = nIndices;
		for (uint16_t i = 0; i < NumIndices; ++i)
		{
			Indices.emplace_back(indices[i]);
		}
	}

	uint16_t	GetNumVertices() const
	{
		return NumVertices;
	}

	uint16_t	GetNumEdges() const
	{
		return NumEdges;
	}

	uint16_t	GetNumFaces() const
	{
		return NumFaces;
	}

	uint16_t	GetNumIndices() const
	{
		return NumIndices;
	}

	Vector3		GetNormal(uint32_t i) const
	{
		return (Vertices[i].p - CenterOfMass).Unit();
	}

	bool		ValidateStructure() const;
	
	bool		BuildHull();
	void		ComputeCenterOfMass();
	void		ComputeInertia();
	void		ComputeBoundingVolume();

	bool		IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;

	const Box3d&	GetBoundingVolume() const
	{
		return BoundingVolume;
	}

	const Matrix3&	GetInertiaTensor(float Mass) const
	{
		return Inertia;
	}

	Vector3		GetSupport(const Vector3& Direction) const;
	int			GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const;

	void		GetMesh(std::vector<Vector3>& _Vertices, std::vector<uint16_t>& _Indices, std::vector<Vector3>& _Normals);
	void		GetWireframe(std::vector<Vector3>& _Vertices, std::vector<uint16_t>& _Indices);
};
