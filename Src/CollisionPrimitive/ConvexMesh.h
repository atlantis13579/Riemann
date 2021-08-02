
#pragma once

#include <vector>

#include "../Maths/Vector3d.h"
#include "../Maths/Box3d.h"
#include "../Maths/Matrix3d.h"
#include "ShapeType.h"
#include "Plane3d.h"

struct HullFace3d
{
	explicit HullFace3d(const Plane3d& p)
	{
		Plane = p;
	}
	Plane3d			Plane;
};

class ConvexMesh
{
public:
	Vector3d					CenterOfMass;
	Box3d						BoundingVolume;
	Matrix3d					Inertia;
	std::vector<Vector3d>		Vertices;
	std::vector<uint16_t>		Edges;
	std::vector<HullFace3d>		Faces;
	uint32_t					NumVertices;
	uint32_t					NumEdges;
	uint32_t					NumFaces;
	
	ConvexMesh()
	{
		Release();
	}

	static constexpr ShapeType	StaticType()
	{
		return ShapeType::CONVEX_MESH;
	}

	void			Release()
	{
		NumVertices = NumFaces = NumEdges = 0;
		Vertices.clear();
		Edges.clear();
		Faces.clear();
	}

	int				EulerNumber() const
	{
		return NumVertices - NumEdges + NumFaces;
	}

	void			AddFace(const Plane3d& p)
	{
		Faces.emplace_back(p);
		NumFaces++;
	}

	void			SetVerties(const Vector3d* Verts, uint32_t Nv)
	{
		Vertices.resize(Nv);
		memcpy(&Vertices[0], Verts, sizeof(Vertices[0]) * Vertices.size());
		NumVertices = Nv;
	}

	void			SetEdges(const uint16_t* Es, uint32_t Ne)
	{
		Edges.resize(Ne * 2);
		memcpy(&Edges[0], Es, sizeof(Edges[0]) * Edges.size());
		NumEdges = Ne;
	}

	uint32_t	GetNumVertices() const
	{
		return NumVertices;
	}

	uint32_t	GetNumEdges() const
	{
		return NumEdges;
	}

	uint32_t	GetNumFaces() const
	{
		return NumFaces;
	}

	Vector3d		GetNormal(uint32_t i) const
	{
		return (Vertices[i] - CenterOfMass).Unit();
	}

	bool			VerifyIndices() const
	{
		for (uint32_t i = 0; i < NumEdges; ++i)
		{
			if (Edges[i] >= NumVertices)
				return false;
		}
		return true;
	}


	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
	{
		// TODO
		return false;
	}

	const Box3d&	GetBoundingVolume() const
	{
		return BoundingVolume;
	}

	const Matrix3d&	GetInertiaTensor(float Mass) const
	{
		return Inertia;
	}

	Vector3d		GetSupport(const Vector3d& dir) const
	{
		// TOOD
		return Vector3d::Zero();
	}

	void	GetMesh(std::vector<Vector3d>& _Vertices, std::vector<uint16_t>& _Indices, std::vector<Vector3d>& _Normals)
	{

	}

	void	GetWireframe(std::vector<Vector3d>& _Vertices, std::vector<uint16_t>& _Indices)
	{
		_Vertices = Vertices;
		_Indices = Edges;
	}
};