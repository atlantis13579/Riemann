
#include "ConvexMesh.h"

void ConvexMesh::SetConvexData(Vector3* verts, uint16_t nVerties,
							   HullFace3d* faces, uint16_t nFaces,
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
		int buffer_size =	sizeof(HullVertex3d) * nVerties +
							sizeof(HullFace3d) * nFaces +
							sizeof(HullEdge3d) * nEdges +
							sizeof(uint8_t) * nIndices;
		
		Buffers.resize(buffer_size);
		
		Vertices = (HullVertex3d*)Buffers.data();
		Faces = (HullFace3d*)(Vertices + nVerties);
		Edges = (HullEdge3d*)(Faces + nFaces);
		Indices = (uint8_t*)(Edges + nEdges);
	}
	else
	{
		Vertices = (HullVertex3d*)verts;
		Faces = faces;
		Edges = (HullEdge3d*)edges;
		Indices = indices;
	}
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

bool ConvexMesh::BuildHull()
{
	ComputeCenterOfMass();
	ComputeBoundingVolume();
	ComputeInertia();
	return ValidateStructure();
}

void ConvexMesh::ComputeCenterOfMass()
{
	CenterOfMass = Vector3::Zero();
	for (uint16_t i = 0; i < NumVertices; ++i)
	{
		CenterOfMass += Vertices[i].p;
	}
	CenterOfMass *= (1.0f / NumVertices);
}

// http://number-none.com/blow/inertia/deriving_i.html
void ConvexMesh::ComputeInertia()
{
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
		covariance_matrix[i][j] /= NumVertices;
	}

	float eigens[3];
	Vector3 eigen_vectors[3];
	covariance_matrix.SolveEigenSymmetric(eigens, eigen_vectors);

	if (Determinant(eigen_vectors[0], eigen_vectors[1], eigen_vectors[2]) < 0)
	{
		eigen_vectors[2] = -eigen_vectors[2];
	}

	Inertia = Matrix3(eigen_vectors[0].x, eigen_vectors[1].x, eigen_vectors[2].x,
					  eigen_vectors[0].y, eigen_vectors[1].y, eigen_vectors[2].y,
					  eigen_vectors[0].z, eigen_vectors[1].z, eigen_vectors[2].z);

	return;
}

void ConvexMesh::ComputeBoundingVolume()
{
	BoundingVolume.SetEmpty();
	for (uint16_t i = 0; i < NumVertices; ++i)
	{
		BoundingVolume.Encapsulate(Vertices[i].p);
	}
}

bool ConvexMesh::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
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

Vector3 ConvexMesh::GetSupport(const Vector3& Direction) const
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

int ConvexMesh::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
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
			FacePoints[j] = Vertices[Indices[hull.first + j]].p;
		}
		return hull.numVerties;
	}

	return 0;
}

void ConvexMesh::GetMesh(std::vector<Vector3>& _Vertices, std::vector<uint16_t>& _Indices, std::vector<Vector3>& _Normals)
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

void ConvexMesh::GetWireframe(std::vector<Vector3>& _Vertices, std::vector<uint16_t>& _Indices)
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
