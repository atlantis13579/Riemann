#include "DeformableBody.h"

#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/TriangleMesh.h"

namespace Riemann
{
	DeformableBody::~DeformableBody()
	{
		delete m_RenderGeometry;
	}

	void DeformableBody::UpdateRenderGeometry()
	{
		const std::vector<Index3>& triangles = GetSurfaceTriangles();
		if (!m_RenderGeometry && triangles.empty()) return;
		if (!m_RenderGeometry)
		{
			m_RenderGeometry = GeometryFactory::CreateTriangleMesh();
			m_RenderGeometry->SetQueryEnabled(false);
			m_RenderGeometry->SetSimulationEnabled(false);
		}
		TriangleMesh* mesh = m_RenderGeometry->GetShapeObj<TriangleMesh>();
		const uint32_t count = static_cast<uint32_t>(GetParticleCount());
		if (m_RenderTopologyDirty || mesh->NumVertices != count || mesh->NumTriangles != triangles.size())
		{
			mesh->Flags = 0;
			mesh->NumVertices = count;
			mesh->NumTriangles = static_cast<uint32_t>(triangles.size());
			mesh->mVertices.resize(count);
			mesh->Vertices = mesh->mVertices.data();
			mesh->mIndices.resize(triangles.size() * 6);
			mesh->Indices = mesh->mIndices.data();
			uint32_t* indices = mesh->GetIndices32();
			for (size_t i = 0; i < triangles.size(); ++i)
			{
				indices[i * 3] = triangles[i].a;
				indices[i * 3 + 1] = triangles[i].b;
				indices[i * 3 + 2] = triangles[i].c;
			}
			m_RenderTopologyDirty = false;
		}
		mesh->BoundingVolume.SetEmpty();
		mesh->mNormals.assign(count, Vector3::Zero());
		for (uint32_t i = 0; i < count; ++i)
		{
			mesh->Vertices[i] = GetParticle(i).Position;
			mesh->BoundingVolume.Encapsulate(mesh->Vertices[i]);
		}
		for (const Index3& triangle : triangles)
		{
			const Vector3 normal = (mesh->Vertices[triangle.b] - mesh->Vertices[triangle.a]).Cross(
				mesh->Vertices[triangle.c] - mesh->Vertices[triangle.a]);
			mesh->mNormals[triangle.a] += normal;
			mesh->mNormals[triangle.b] += normal;
			mesh->mNormals[triangle.c] += normal;
		}
		for (Vector3& normal : mesh->mNormals) normal = normal.SafeUnit();
		m_RenderGeometry->SetShapeBounds(mesh->BoundingVolume);
	}
}
