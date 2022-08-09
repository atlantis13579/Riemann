
#pragma once

#include <assert.h>
#include <stdint.h>
#include <vector>

#include "ShapeType.h"
#include "Plane3d.h"
#include "../Maths/Vector3.h"
#include "../Maths/Box3d.h"
#include "../Maths/Matrix3.h"


struct HullFace3d
{
	explicit HullFace3d(const Plane3d& p, uint8_t nVerties, uint16_t first_idx)
	{
		Plane = p;
		NumVerties = nVerties;
		FirstVertex = first_idx;
	}
	Plane3d			Plane;
	uint8_t			NumVerties;
	uint16_t		FirstVertex;
};

struct HullEdge
{
	explicit HullEdge(uint16_t _s, uint16_t _e)
	{
		s = _s;
		e = _e;
	}
	uint16_t		s;
	uint16_t		e;
};

class ConvexMesh
{
public:
	Vector3						CenterOfMass;
	Box3d						BoundingVolume;
	Matrix3						Inertia;
	std::vector<Vector3>		Vertices;
	std::vector<HullEdge>		Edges;
	std::vector<HullFace3d>		Faces;
	uint32_t					NumVertices;
	uint32_t					NumEdges;
	uint32_t					NumFaces;
	
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

	void		SetVerties(const Vector3* Verts, uint32_t nVerties)
	{
		Vertices.resize(nVerties);
		memcpy(&Vertices[0], Verts, sizeof(Vertices[0]) * Vertices.size());
		NumVertices = nVerties;
	}

	void		SetEdges(const uint16_t* edges, uint32_t nEdges)
	{
		NumEdges = nEdges;
		for (uint32_t i = 0; i < nEdges; i ++)
		{
			Edges.emplace_back(edges[2 * i], edges[2 * i + 1]);
		}
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

	Vector3		GetNormal(uint32_t i) const
	{
		return (Vertices[i] - CenterOfMass).Unit();
	}

	bool		VerifyIndices() const
	{
		if (NumEdges != (int)Edges.size())
			return false;
		for (uint32_t i = 0; i < Edges.size(); ++i)
		{
			if (Edges[i].s >= NumVertices)
				return false;
			if (Edges[i].e >= NumVertices)
				return false;
		}
		return true;
	}

	// http://number-none.com/blow/inertia/deriving_i.html
	void		CalcInertia()
	{
		Vector3 mean;
		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			mean += Vertices[i];
		}
		mean *= (1.0f / Vertices.size());

		Matrix3 covariance_matrix;
		covariance_matrix.LoadZero();

		for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
		{
			for (size_t k = 0; k < Vertices.size(); ++k)
			{
				float cij = (Vertices[k][i] - mean[i]) * (Vertices[k][j] - mean[j]);
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

	void		BuildBoundingVolume()
	{
		BoundingVolume.SetEmpty();
		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			BoundingVolume.Encapsulate(Vertices[i]);
		}
	}

	bool		IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		int plane_test = 0;
		bool all_inside = true;
		float min_t = 0.0f, max_t = 1.0f;
		for (size_t i = 0; i < Faces.size(); ++i)
		{
			const Plane3d& p = Faces[i].Plane;

			Vector3 normal = p.Normal;
			float dist = Origin.Dot(normal) + p.D;
			bool is_outside = dist > 0.0f;
			if (is_outside)
			{
				all_inside = false;
			}

			float proj_dist = Direction.Dot(normal);
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

		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			float dot = Vertices[i].Dot(Direction);
			if (dot > max_dot)
			{
				max_dot = dot;
				max_v = Vertices[i];
			}
		}

		return max_v;
	}

	int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
	{
		assert(false);
		return 0;
	}

	void	GetMesh(std::vector<Vector3>& _Vertices, std::vector<uint16_t>& _Indices, std::vector<Vector3>& _Normals)
	{
		_Vertices = Vertices;
		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			_Normals.push_back(GetNormal((uint32_t)i));
		}

		for (size_t i = 0; i < Faces.size(); ++i)
		{
			const HullFace3d& face = Faces[i];
			for (uint8_t j = 1; j < face.NumVerties - 1; ++j)
			{
				_Indices.push_back(face.FirstVertex);
				_Indices.push_back(face.FirstVertex + j);
				_Indices.push_back(face.FirstVertex + j + 1);
			}
		}
	}

	void	GetWireframe(std::vector<Vector3>& _Vertices, std::vector<uint16_t>& _Indices)
	{
		_Vertices = Vertices;
		for (size_t i = 0; i < Edges.size(); ++i)
		{
			_Indices.push_back(Edges[i].s);
			_Indices.push_back(Edges[i].e);
		}
	}
};
