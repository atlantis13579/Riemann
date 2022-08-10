
#include "Polyhedron.h"

template<>
float 	Tetrahedron::GetVolume() const
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
