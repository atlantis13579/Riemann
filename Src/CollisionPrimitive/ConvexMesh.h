
#pragma once

#include <vector>

#include "../Maths/Vector3d.h"
#include "../Maths/Box3d.h"
#include "../Maths/Matrix3d.h"
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
	Vector3d						CenterOfMass;
	Box3d							BoundingVolume;
	Matrix3d						Inertia;
	std::vector<Vector3d>			Vertices;
	std::vector<unsigned short>		Edges;
	std::vector<HullFace3d>			Faces;
	unsigned int					NumVertices;
	unsigned int					NumEdges;
	unsigned int					NumFaces;
	
	ConvexMesh()
	{
		Release();
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

	void			SetVerties(const Vector3d* Verts, unsigned int Nv)
	{
		Vertices.resize(Nv);
		memcpy(&Vertices[0], Verts, sizeof(Vertices[0]) * Vertices.size());
		NumVertices = Nv;
	}

	void			SetEdges(const unsigned short* Es, unsigned int Ne)
	{
		Edges.resize(Ne * 2);
		memcpy(&Edges[0], Es, sizeof(Edges[0]) * Edges.size());
		NumEdges = Ne;
	}

	unsigned int	GetNumVerties() const
	{
		return NumVertices;
	}

	unsigned int	GetNumEdges() const
	{
		return NumEdges;
	}

	unsigned int	GetNumFaces() const
	{
		return NumFaces;
	}

	Vector3d		GetNormal(unsigned int i) const
	{
		return (Vertices[i] - CenterOfMass).Unit();
	}

	bool			VerifyIndices() const
	{
		for (unsigned int i = 0; i < NumEdges; ++i)
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

};