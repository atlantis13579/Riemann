#include "DeformableBody.h"

#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/TriangleMesh.h"
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	DeformableBody::~DeformableBody()
	{
		DisconnectAttachments();
		delete m_RenderGeometry;
	}

	void DeformableBody::UpdateRenderGeometry()
	{
		const bool Embedded = IsEmbeddedRenderMeshEnabled();
		const std::vector<Index3>& triangles = Embedded ? m_Asset.GetRenderTriangles() : GetSurfaceTriangles();
		if (!m_RenderGeometry && triangles.empty()) return;
		if (!m_RenderGeometry)
		{
			m_RenderGeometry = GeometryFactory::CreateTriangleMesh();
			m_RenderGeometry->SetQueryEnabled(false);
			m_RenderGeometry->SetSimulationEnabled(false);
		}
		TriangleMesh* mesh = m_RenderGeometry->GetShapeObj<TriangleMesh>();
		const uint32_t count = static_cast<uint32_t>(Embedded ? m_Asset.GetRenderBindings().size() : GetParticleCount());
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
		// Embedded render vertices may lie inside the simulated tetrahedral surface.
		if (Embedded) for (const auto& P : m_Particles) mesh->BoundingVolume.Encapsulate(P.Position);
		mesh->mNormals.assign(count, Vector3::Zero());
		for (uint32_t i = 0; i < count; ++i)
		{
			if (!Embedded) mesh->Vertices[i] = GetParticle(i).Position;
			else
			{
				const auto& Binding = m_Asset.GetRenderBindings()[i]; const auto& E = m_Asset.GetElements()[Binding.Element];
				const auto& Origin = m_Particles[E.Vertices[0]].Position;
				DeformationGradient::DVector P(Origin.x, Origin.y, Origin.z);
				for (int J = 1; J < 4; ++J)
				{
					const auto& V = m_Particles[E.Vertices[J]].Position;
					P += DeformationGradient::DVector((double)V.x - Origin.x, (double)V.y - Origin.y, (double)V.z - Origin.z) * Binding.Weights[J];
				}
				mesh->Vertices[i] = Vector3((float)P.x, (float)P.y, (float)P.z);
				if (!std::isfinite(mesh->Vertices[i].x) || !std::isfinite(mesh->Vertices[i].y) || !std::isfinite(mesh->Vertices[i].z)) throw std::overflow_error("Embedded render position overflow");
			}
			mesh->BoundingVolume.Encapsulate(mesh->Vertices[i]);
		}
		m_RenderNormals.assign(count, DeformationGradient::DVector(0));
		for (const Index3& triangle : triangles)
		{
			const auto& A = mesh->Vertices[triangle.a]; const auto& B = mesh->Vertices[triangle.b]; const auto& C = mesh->Vertices[triangle.c];
			const auto Normal = DeformationGradient::DVector((double)B.x - A.x, (double)B.y - A.y, (double)B.z - A.z)
				.Cross(DeformationGradient::DVector((double)C.x - A.x, (double)C.y - A.y, (double)C.z - A.z));
			m_RenderNormals[triangle.a] += Normal; m_RenderNormals[triangle.b] += Normal; m_RenderNormals[triangle.c] += Normal;
		}
		for (size_t I = 0; I < m_RenderNormals.size(); ++I)
		{
			const auto& N = m_RenderNormals[I]; const double Length = std::hypot(std::hypot(N.x, N.y), N.z);
			mesh->mNormals[I] = Length > 0 ? Vector3((float)(N.x / Length), (float)(N.y / Length), (float)(N.z / Length)) : Vector3::Zero();
		}
		m_RenderGeometry->SetShapeBounds(mesh->BoundingVolume);
	}
}
