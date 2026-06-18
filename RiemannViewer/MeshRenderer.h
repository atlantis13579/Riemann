#pragma once

#include <string>
#include <vector>

#include "../RiemannRenderer/RiemannRenderer.h"
#include "../Src/Maths/Box3.h"

namespace Riemann
{
	class Geometry;
	class StaticMesh;

	void BuildGeometryMeshes(Geometry* geometry, const std::string& id, const Vector4& color, bool renderBounds, std::vector<RenderMeshDesc>* meshes);
	void BuildTriMeshMeshes(StaticMesh* mesh, const Transform& transform, const std::string& id, const Vector4& color, bool renderBounds, std::vector<RenderMeshDesc>* meshes);
	void BuildAabbWireMesh(const Box3& aabb, const std::string& id, const Vector4& color, std::vector<RenderMeshDesc>* meshes);
}
