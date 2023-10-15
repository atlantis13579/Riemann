
#include "Mesh.h"
#include "../Geometry/MeshSimplification.h"

bool Mesh::Simplify(float rate)
{
	std::vector<Vector3> new_v;
	std::vector<int> new_i;
	if (!SimplifyMesh((Vector3*)GetVertexBuffer(), GetIndexBuffer(), GetNumVertices(), GetNumTriangles(), Is16bitIndices(), rate, new_v, new_i))
	{
		return false;
	}

	SetData(new_v.data(), new_i.data(), (uint32_t)new_v.size(), (uint32_t)new_i.size() / 3, false, true);
	Compact();
	
	return true;
}