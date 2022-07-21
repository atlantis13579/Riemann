#include "Renderer.h"
#include <vector>
#include <string>
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/AxisAlignedBox3d.h"
#include "../Src/CollisionPrimitive/Sphere3d.h"
#include "../Src/CollisionPrimitive/Plane3d.h"
#include "../Src/CollisionPrimitive/Cylinder3d.h"
#include "../Src/CollisionPrimitive/Capsule3d.h"
#include "../Src/CollisionPrimitive/Mesh.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/CollisionPrimitive/HeightField3d.h"

void Renderer::AddPlane(Geometry* geom, bool DrawMesh )
{
	std::vector<Vector3d> Vertices;
	std::vector<uint16_t> Indices;
	std::vector<Vector3d> Normals;
	Plane3d* shape = geom->GetShapeObj<Plane3d>();

	std::string name = std::to_string(geom->GetGuid());
	if (DrawMesh)
	{
		shape->GetMesh(Vertices, Indices, Normals);
		std::vector<Vertex1> vv;
		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			vv.emplace_back(Vertices[i], Vector3d::One());
		}
		AddTriangles(name.c_str(), geom->GetTransform(), &vv[0], (int)vv.size(), &Indices[0], (int)Indices.size(), 2);
	}
}

template <class TShape>
void Renderer::AddGeometry(Geometry* geom, bool DrawMesh)
{
	std::vector<Vector3d> Vertices;
	std::vector<uint16_t> Indices;
	std::vector<Vector3d> Normals;
	TShape* shape = geom->GetShapeObj<TShape>();

	std::string name = std::to_string(geom->GetGuid());
	if (DrawMesh)
	{
		shape->GetMesh(Vertices, Indices, Normals);
		std::vector<Vertex1> vv;
		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			vv.emplace_back(Vertices[i], Normals[i]);
		}
		AddTriangles(name.c_str(), geom->GetTransform(), &vv[0], (int)vv.size(), &Indices[0], (int)Indices.size(), 2);
	}
	else
	{
		shape->GetWireframe(Vertices, Indices);
		std::vector<Vertex1> vv;
		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			vv.emplace_back(Vertices[i], Vector3d(1.0f, 1.0f, 1.0f));
		}
		AddWireframe(name.c_str(), geom->GetTransform(), &vv[0], (int)vv.size(), &Indices[0], (int)Indices.size());
	}
}

void Renderer::AddTriMesh(Mesh* mesh, Transform* Trans, bool RenderBV)
{
	mesh->CalculateNormals();

	std::vector<Vertex1> vv;
	for (uint32_t i = 0; i < mesh->GetNumVertices(); ++i)
	{
		vv.emplace_back(mesh->Vertices[i], mesh->mNormals[i]);
	}

	AddTriangles(mesh->ResourceId.c_str(), Trans, &vv[0], (int)vv.size(), mesh->GetIndexBuffer(), mesh->GetNumTriangles() * 3, mesh->GetIndicesWidth() * 2);

	if (RenderBV)
	{
		std::vector<Vector3d> Vertices;
		std::vector<uint16_t> Indices;
		AxisAlignedBox3d aabb(mesh->BoundingVolume.Min, mesh->BoundingVolume.Max);
		aabb.GetWireframe(Vertices, Indices);
		vv.clear();
		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			vv.emplace_back(Vertices[i], Vector3d(1.0f, 1.0f, 1.0f));
		}
		AddWireframe(mesh->ResourceId.c_str(), Trans, &vv[0], (int)vv.size(), &Indices[0], (int)Indices.size());
	}

	return;
}

void Renderer::AddGeometry(Geometry* geom)
{
	if (geom->GetShapeType() == ShapeType3d::TRIANGLE_MESH)
	{
		AddTriMesh(geom->GetShapeObj<Mesh>(), geom->GetTransform(), true);
	}
	else if (geom->GetShapeType() == ShapeType3d::CONVEX_MESH)
	{
		AddGeometry<ConvexMesh>(geom);
	}
	else if (geom->GetShapeType() == ShapeType3d::BOX)
	{
		AddGeometry<AxisAlignedBox3d>(geom);
	}
	else if (geom->GetShapeType() == ShapeType3d::PLANE)
	{
		AddPlane(geom);
	}
	else if (geom->GetShapeType() == ShapeType3d::SPHERE)
	{
		AddGeometry <Sphere3d >(geom);
	}
	else if (geom->GetShapeType() == ShapeType3d::CAPSULE)
	{
		AddGeometry <Capsule3d >(geom);
	}
	else if (geom->GetShapeType() == ShapeType3d::CYLINDER)
	{
		AddGeometry <Cylinder3d >(geom);
	}
	else if (geom->GetShapeType() == ShapeType3d::HEIGHTFIELD)
	{
		AddGeometry <HeightField3d >(geom);
	}
}
