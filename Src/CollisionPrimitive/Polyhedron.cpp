
#include <assert.h>
#include "Polyhedron.h"

#include "../Geometry/ConvexHull3d.h"

template<>
float 	Tetrahedron::CalculateVolume() const
{
	float det = Determinant(v[0] - v[3], v[1] - v[3], v[2] - v[3]);
	return fabsf(det);
}

template<>
void	Tetrahedron::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
}

template<>
void	Tetrahedron::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
{
	Vertices = std::vector<Vector3>({ v[0], v[1], v[2], v[3] });
	Indices = std::vector<uint16_t>({ 0,1, 1,2, 2,0, 0,3, 1,3, 2,3 });
}

Matrix3 computeRegularPolyhedronInertiaTensor(int V, int F, int P, const Vector3& cm, Vector3* verts, uint8_t* indices)
{
	ConvexHull3d hull;
	hull.faces.resize(F);
	hull.verts.reserve(V);

	for (uint16_t i = 0; i < V; ++i)
	{
		hull.verts[i] = verts[i];
	}
	for (uint16_t i = 0; i < F; ++i)
	{
		uint8_t i0 = i * P;

		assert(P >= 3);
		Vector3 normal = CrossProduct(verts[indices[i0+1]] - verts[indices[i0]], verts[indices[i0+2]] - verts[indices[i0]]);
		Vector3 dir = verts[indices[i0]] - cm;
		if (normal.Dot(dir) < 0)
			normal = -normal.Unit();
		else
			normal = normal.Unit();

		HullFace3d& face = hull.faces[i];
		face.norm = normal;
		face.w = -verts[indices[i0]].Dot(normal);
		face.verts.resize(P);
		for (int j = 0; j < P; ++j)
		{
			face.verts[j] = indices[i0 + j];
		}
	}

	float Mass, Volume;
	Vector3 center_of_mass;
	return ComputePolyhedralInertiaTensor_VolumeIntegration(hull, 1.0f, Volume, Mass, center_of_mass);
}
