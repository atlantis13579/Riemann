#include "MeshRenderer.h"

#include <stdint.h>
#include <string>
#include <vector>

#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/AxisAlignedBox3.h"
#include "../Src/CollisionPrimitive/Capsule3.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/CollisionPrimitive/Cylinder3.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/CollisionPrimitive/Plane3.h"
#include "../Src/CollisionPrimitive/Sphere3.h"
#include "../Src/CollisionPrimitive/StaticMesh.h"

namespace Riemann
{
	namespace
	{
		void AppendIndices(const std::vector<uint16_t>& src, std::vector<uint32_t>* dst)
		{
			dst->reserve(src.size());
			for (uint16_t index : src)
			{
				dst->push_back(static_cast<uint32_t>(index));
			}
		}

		RenderMeshDesc MakeMeshDesc(
			const std::string& id,
			const Transform& transform,
			const std::vector<Vertex1>& vertices,
			const std::vector<uint32_t>& indices,
			const Vector4& color,
			RenderPrimitiveTopology topology,
			bool castShadow)
		{
			RenderMeshDesc mesh;
			mesh.Id = id;
			mesh.WorldTransform = transform;
			mesh.Vertices = vertices;
			mesh.Indices = indices;
			mesh.Color = color;
			mesh.Topology = topology;
			mesh.CastShadow = castShadow;
			return mesh;
		}

		void BuildPlaneMesh(Geometry* geometry, const std::string& id, const Vector4& color, std::vector<RenderMeshDesc>* meshes)
		{
			Plane3* shape = geometry->GetShapeObj<Plane3>();
			if (shape == nullptr)
			{
				return;
			}

			std::vector<Vector3> positions;
			std::vector<uint16_t> indices16;
			std::vector<Vector3> normals;
			shape->GetMesh(positions, indices16, normals);

			std::vector<Vertex1> vertices;
			vertices.reserve(positions.size());
			for (size_t i = 0; i < positions.size(); ++i)
			{
				const Vector3 normal = i < normals.size() ? normals[i] : Vector3::UnitY();
				vertices.push_back(Vertex1(positions[i], normal));
			}

			std::vector<uint32_t> indices;
			AppendIndices(indices16, &indices);
			meshes->push_back(MakeMeshDesc(id, *geometry->GetWorldTransform(), vertices, indices, color, RenderPrimitiveTopology::Triangles, true));
		}

		template <class TShape>
		void BuildGeometryMeshImpl(Geometry* geometry, const std::string& id, const Vector4& color, std::vector<RenderMeshDesc>* meshes)
		{
			TShape* shape = geometry->GetShapeObj<TShape>();
			if (shape == nullptr)
			{
				return;
			}

			std::vector<Vector3> positions;
			std::vector<uint16_t> indices16;
			std::vector<Vector3> normals;
			shape->GetMesh(positions, indices16, normals);

			std::vector<Vertex1> vertices;
			vertices.reserve(positions.size());
			for (size_t i = 0; i < positions.size(); ++i)
			{
				const Vector3 normal = i < normals.size() ? normals[i] : Vector3::UnitY();
				vertices.push_back(Vertex1(positions[i], normal));
			}

			std::vector<uint32_t> indices;
			AppendIndices(indices16, &indices);
			meshes->push_back(MakeMeshDesc(id, *geometry->GetWorldTransform(), vertices, indices, color, RenderPrimitiveTopology::Triangles, true));
		}

		void BuildAabbWireMesh(const AxisAlignedBox3& aabb, const Transform& transform, const std::string& id, std::vector<RenderMeshDesc>* meshes)
		{
			std::vector<Vector3> positions;
			std::vector<uint16_t> indices16;
			AxisAlignedBox3 wireBox = aabb;
			wireBox.GetWireframe(positions, indices16);

			std::vector<Vertex1> vertices;
			vertices.reserve(positions.size());
			for (const Vector3& position : positions)
			{
				vertices.push_back(Vertex1(position, Vector3::UnitY()));
			}

			std::vector<uint32_t> indices;
			AppendIndices(indices16, &indices);
			meshes->push_back(MakeMeshDesc(id, transform, vertices, indices, Vector4(1.0f, 1.0f, 1.0f, 1.0f), RenderPrimitiveTopology::Lines, false));
		}

		void AppendStaticMeshIndices(StaticMesh* mesh, std::vector<uint32_t>* indices)
		{
			const uint32_t indexCount = mesh->GetTriangleCount() * 3;
			indices->reserve(indexCount);
			if (mesh->Is16bitIndices())
			{
				const uint16_t* src = mesh->GetIndices16();
				for (uint32_t i = 0; i < indexCount; ++i)
				{
					indices->push_back(static_cast<uint32_t>(src[i]));
				}
			}
			else
			{
				const uint32_t* src = mesh->GetIndices32();
				for (uint32_t i = 0; i < indexCount; ++i)
				{
					indices->push_back(src[i]);
				}
			}
		}
	}

	void BuildTriMeshMeshes(StaticMesh* mesh, const Transform& transform, const std::string& id, const Vector4& color, bool renderBounds, std::vector<RenderMeshDesc>* meshes)
	{
		if (mesh == nullptr || meshes == nullptr || mesh->GetVertexCount() == 0 || mesh->GetTriangleCount() == 0)
		{
			return;
		}

		mesh->CalculateWeightAverageNormals();

		std::vector<Vertex1> vertices;
		vertices.reserve(mesh->GetVertexCount());
		for (uint32_t i = 0; i < mesh->GetVertexCount(); ++i)
		{
			const Vector3 normal = i < mesh->mNormals.size() ? mesh->mNormals[i] : Vector3::UnitY();
			vertices.push_back(Vertex1(mesh->Vertices[i], normal));
		}

		std::vector<uint32_t> indices;
		AppendStaticMeshIndices(mesh, &indices);
		meshes->push_back(MakeMeshDesc(id, transform, vertices, indices, color, RenderPrimitiveTopology::Triangles, true));

		if (renderBounds)
		{
			BuildAabbWireMesh(AxisAlignedBox3(mesh->BoundingVolume.Min, mesh->BoundingVolume.Max), transform, id + ".bounds", meshes);
		}
	}

	void BuildGeometryMeshes(Geometry* geometry, const std::string& id, const Vector4& color, bool renderBounds, std::vector<RenderMeshDesc>* meshes)
	{
		if (geometry == nullptr || meshes == nullptr)
		{
			return;
		}

		if (geometry->GetShapeType() == PrimitiveType::TRIANGLE_MESH)
		{
			BuildTriMeshMeshes(geometry->GetShapeObj<StaticMesh>(), *geometry->GetWorldTransform(), id, color, renderBounds, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::CONVEX_MESH)
		{
			BuildGeometryMeshImpl<ConvexMesh>(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::BOX)
		{
			BuildGeometryMeshImpl<AxisAlignedBox3>(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::PLANE)
		{
			BuildPlaneMesh(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::SPHERE)
		{
			BuildGeometryMeshImpl<Sphere3>(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::CAPSULE)
		{
			BuildGeometryMeshImpl<Capsule3>(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::CYLINDER)
		{
			BuildGeometryMeshImpl<Cylinder3>(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::HEIGHTFIELD)
		{
			BuildGeometryMeshImpl<HeightField3>(geometry, id, color, meshes);
		}
	}
}
