
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

	bool		ValidateStructure() const
	{
		if (NumFaces != (int)Faces.size())
		{
			return false;
		}
		
		if (NumVertices != (int)Vertices.size())
		{
			return false;
		}
		
		if (NumEdges != (int)Edges.size())
		{
			return false;
		}

		if (NumIndices != (int)Indices.size())
		{
			return false;
		}
		
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
	
	bool		BuildHull()
	{
		ComputeCenterOfMass();
		ComputeBoundingVolume();
		ComputeInertia();
		return ValidateStructure();
	}
	
	void		ComputeCenterOfMass()
	{
		CenterOfMass = Vector3::Zero();
		for (uint16_t i = 0; i < NumVertices; ++i)
		{
			CenterOfMass += Vertices[i].p;
		}
		CenterOfMass *= (1.0f / Vertices.size());
	}

	// http://number-none.com/blow/inertia/deriving_i.html
	void		ComputeInertia()
	{
		Vector3 mean;
		for (uint16_t i = 0; i < NumVertices; ++i)
		{
			mean += Vertices[i].p;
		}
		mean *= (1.0f / Vertices.size());

		Matrix3 covariance_matrix;
		covariance_matrix.LoadZero();

		for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
		{
			for (uint16_t k = 0; k < NumVertices; ++k)
			{
				float cij = (Vertices[k].p[i] - CenterOfMass[i]) * (Vertices[k].p[j] - CenterOfMass[j]);
				covariance_matrix[i][j] += cij;
			}
			covariance_matrix[i][j] /= Vertices.size();
		}

		float eigens[3];
		Vector3 eigen_vectors[3];
		covariance_matrix.SolveEigenSymmetric(eigens, eigen_vectors);

		if (Determinant(eigen_vectors[0], eigen_vectors[1], eigen_vectors[2]) < 0)
		{
			eigen_vectors[2] = -eigen_vectors[2];
		}

		Inertia = Matrix3(	eigen_vectors[0].x, eigen_vectors[1].x, eigen_vectors[2].x,
							eigen_vectors[0].y, eigen_vectors[1].y, eigen_vectors[2].y,
							eigen_vectors[0].z, eigen_vectors[1].z, eigen_vectors[2].z);
		
		return;
	}

	void		ComputeBoundingVolume()
	{
		BoundingVolume.SetEmpty();
		for (uint16_t i = 0; i < NumVertices; ++i)
		{
			BoundingVolume.Encapsulate(Vertices[i].p);
		}
	}

	bool		IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		int plane_test = 0;
		bool all_inside = true;
		float min_t = 0.0f, max_t = 1.0f;
		for (uint16_t i = 0; i < NumFaces; ++i)
		{
			const Plane3d& p = Faces[i].plane;

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

	const Box3d&	GetBoundingVolume() const
	{
		return BoundingVolume;
	}

	const Matrix3&	GetInertiaTensor(float Mass) const
	{
		return Inertia;
	}

	Vector3		GetSupport(const Vector3& Direction) const
	{
		float max_dot = -FLT_MAX;
		Vector3 max_v = Vector3::Zero();

		for (uint16_t i = 0; i < NumVertices; ++i)
		{
			const float dot = Vertices[i].p.Dot(Direction);
			if (dot > max_dot)
			{
				max_dot = dot;
				max_v = Vertices[i].p;
			}
		}

		return max_v;
	}

	int			GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
	{
		float max_dp = -FLT_MAX;
		uint16_t max_idx = 0xFFFF;
		for (uint16_t i = 0; i < NumFaces; ++i)
		{
			const Plane3d& p = Faces[i].plane;
			const float dp = Direction.Dot(p.Normal);
			if (dp > max_dp)
			{
				max_dp = dp;
				max_idx = i;
			}
		}

		if (max_idx != 0xFFFF)
		{
			const HullFace3d& hull = Faces[max_idx];
			for (uint8_t j = 0; j < hull.numVerties; ++j)
			{
				FacePoints[j++] = Vertices[Indices[hull.first + j]].p;
			}
			return hull.numVerties;
		}

		return 0;
	}

	void		GetMesh(std::vector<Vector3>& _Vertices, std::vector<uint16_t>& _Indices, std::vector<Vector3>& _Normals)
	{
		for (uint16_t i = 0; i < NumVertices; ++i)
		{
			_Vertices.push_back(Vertices[i].p);
			_Normals.push_back(GetNormal((uint32_t)i));
		}

		for (uint16_t i = 0; i < NumFaces; ++i)
		{
			const HullFace3d& face = Faces[i];
			for (uint8_t j = 1; j < face.numVerties - 1; ++j)
			{
				_Indices.push_back(Indices[face.first]);
				_Indices.push_back(Indices[face.first + j]);
				_Indices.push_back(Indices[face.first + j + 1]);
			}
		}
	}

	void		GetWireframe(std::vector<Vector3>& _Vertices, std::vector<uint16_t>& _Indices)
	{
		for (uint16_t i = 0; i < NumVertices; ++i)
		{
			_Vertices.push_back(Vertices[i].p);
		}
		for (uint16_t i = 0; i < NumEdges; ++i)
		{
			_Indices.push_back(Edges[i].s);
			_Indices.push_back(Edges[i].e);
		}
	}
};
