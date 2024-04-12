#include <vector>
#include <string>
#include "../Renderer/Renderer.h"
#include "ObjRenderer.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/AxisAlignedBox3.h"
#include "../Src/CollisionPrimitive/Sphere3.h"
#include "../Src/CollisionPrimitive/Plane3.h"
#include "../Src/CollisionPrimitive/Cylinder3.h"
#include "../Src/CollisionPrimitive/Capsule3.h"
#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/CollisionPrimitive/HeightField3.h"

namespace Riemann
{
	void AddPlane(Renderer* renderer, GeometryBase* geom, bool DrawMesh = true)
	{
		std::vector<Vector3> Vertices;
		std::vector<uint16_t> Indices;
		std::vector<Vector3> Normals;
		Plane3* shape = geom->GetShapeObj<Plane3>();

		std::string name = std::to_string((intptr_t)geom);
		if (DrawMesh)
		{
			shape->GetMesh(Vertices, Indices, Normals);
			std::vector<Vertex1> vv;
			for (size_t i = 0; i < Vertices.size(); ++i)
			{
				vv.emplace_back(Vertices[i], Vector3::One());
			}
			renderer->AddTriangles(name.c_str(), geom->GetWorldTransform(), &vv[0], (int)vv.size(), &Indices[0], (int)Indices.size(), 2);
		}
	}

	template <class TShape>
	void AddGeometryImpl(Renderer* renderer, GeometryBase* geom, bool DrawMesh = true)
	{
		std::vector<Vector3> Vertices;
		std::vector<uint16_t> Indices;
		std::vector<Vector3> Normals;
		TShape* shape = geom->GetShapeObj<TShape>();

		std::string name = std::to_string((intptr_t)geom);
		if (DrawMesh)
		{
			shape->GetMesh(Vertices, Indices, Normals);
			std::vector<Vertex1> vv;
			for (size_t i = 0; i < Vertices.size(); ++i)
			{
				vv.emplace_back(Vertices[i], Normals[i]);
			}
			renderer->AddTriangles(name.c_str(), geom->GetWorldTransform(), &vv[0], (int)vv.size(), &Indices[0], (int)Indices.size(), 2);
		}
		else
		{
			shape->GetWireframe(Vertices, Indices);
			std::vector<Vertex1> vv;
			for (size_t i = 0; i < Vertices.size(); ++i)
			{
				vv.emplace_back(Vertices[i], Vector3(1.0f, 1.0f, 1.0f));
			}
			renderer->AddWireframe(name.c_str(), geom->GetWorldTransform(), &vv[0], (int)vv.size(), &Indices[0], (int)Indices.size());
		}
	}

	void AddTriMesh(Renderer* renderer, StaticMesh* mesh, void* Trans, bool RenderBV)
	{
		mesh->CalculateNormals();

		std::vector<Vertex1> vv;
		for (uint32_t i = 0; i < mesh->GetVertexCount(); ++i)
		{
			vv.emplace_back(mesh->Vertices[i], mesh->mNormals[i]);
		}

		renderer->AddTriangles(mesh->ResourceId.c_str(), Trans, &vv[0], (int)vv.size(), mesh->GetIndexBuffer(), mesh->GetTriangleCount() * 3, mesh->GetIndicesWidth() * 2);

		if (RenderBV)
		{
			std::vector<Vector3> Vertices;
			std::vector<uint16_t> Indices;
			AxisAlignedBox3 aabb(mesh->BoundingVolume.Min, mesh->BoundingVolume.Max);
			aabb.GetWireframe(Vertices, Indices);
			vv.clear();
			for (size_t i = 0; i < Vertices.size(); ++i)
			{
				vv.emplace_back(Vertices[i], Vector3(1.0f, 1.0f, 1.0f));
			}
			renderer->AddWireframe(mesh->ResourceId.c_str(), Trans, &vv[0], (int)vv.size(), &Indices[0], (int)Indices.size());
		}

		return;
	}

	void AddGeometry(Renderer* renderer, GeometryBase* geom)
	{
		if (geom->GetShapeType() == ShapeType::TRIANGLE_MESH)
		{
			AddTriMesh(renderer, geom->GetShapeObj<StaticMesh>(), geom->GetWorldTransform(), true);
		}
		else if (geom->GetShapeType() == ShapeType::CONVEX_MESH)
		{
			AddGeometryImpl<ConvexMesh>(renderer, geom);
		}
		else if (geom->GetShapeType() == ShapeType::BOX)
		{
			AddGeometryImpl<AxisAlignedBox3>(renderer, geom);
		}
		else if (geom->GetShapeType() == ShapeType::PLANE)
		{
			AddPlane(renderer, geom);
		}
		else if (geom->GetShapeType() == ShapeType::SPHERE)
		{
			AddGeometryImpl <Sphere3 >(renderer, geom);
		}
		else if (geom->GetShapeType() == ShapeType::CAPSULE)
		{
			AddGeometryImpl <Capsule3 >(renderer, geom);
		}
		else if (geom->GetShapeType() == ShapeType::CYLINDER)
		{
			AddGeometryImpl <Cylinder3 >(renderer, geom);
		}
		else if (geom->GetShapeType() == ShapeType::HEIGHTFIELD)
		{
			AddGeometryImpl <HeightField3 >(renderer, geom);
		}
	}
}