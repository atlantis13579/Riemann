#pragma once

#ifdef RENDERER_EXPORT
#  define RENDERER_API __declspec(dllexport)
#else
#  define RENDERER_API __declspec(dllimport)
#endif

#include "../Src/Maths/Vector3d.h"
#include "../Src/Maths/Vector4d.h"

class Mesh;
class ConvexMesh;
class Transform;
class Geometry;

struct Vertex1
{
	Vertex1() {}
	Vertex1(const Vector3d& _p, const Vector3d& _n)
	{
		Pos = _p;
		Normal = _n;
	}
	Vector3d Pos;
	Vector3d Normal;
};

class RENDERER_API Renderer
{
public:
	Renderer() {}
	virtual ~Renderer() {}

	virtual void Render() = 0;

	virtual void SetCameraLookAt(Vector3d Eye, Vector3d At) = 0;
	virtual bool AddTriangles(const char* Id, Transform* pTrans, const Vertex1* pVerties, int nVerties, const void* pIndices, int nIndices, int IndicesWidth) = 0;
	virtual bool AddWireframe(const char* Id, Transform* pTrans, const Vertex1* pVerties, int nVerties, const void* pIndices, int nIndices) = 0;
	virtual bool UpdateVerties(const char* Id, const Vertex1* pVerties, int nVerties) = 0;
	virtual bool DeleteMesh(const char* Id) = 0;

	virtual void SetFillMode(bool Wireframe) = 0;
	virtual void SetDepthMode() = 0;

	virtual void AddGeometry(Geometry* geom) = 0;
	virtual void AddTriMesh(Mesh* mesh, Transform* Trans) = 0;

	static Renderer* CreateDX11Renderer(void* hWnd, const char* shader_path);
};

