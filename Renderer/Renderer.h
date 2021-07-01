#pragma once

#ifdef RENDERER_EXPORT
#  define RENDERER_API __declspec(dllexport)
#else
#  define RENDERER_API __declspec(dllimport)
#endif

#include "../Src/Maths/Vector3d.h"
#include "../Src/Maths/Vector4d.h"

struct Vertex1
{
	Vector3d Pos;
	Vector4d Color;
};

class RENDERER_API Renderer
{
public:
	Renderer() {}
	virtual ~Renderer() {}

	virtual void Render() = 0;

	virtual void SetCameraLookAt(Vector3d Eye, Vector3d At) = 0;
	virtual bool AddMesh(const char* Id, const Vertex1* pVerties, int nVerties, const unsigned short* pIndices, int nIndices) = 0;
	virtual bool UpdateVerties(const char* Id, const Vertex1* pVerties, int nVerties) = 0;
	virtual bool DeleteMesh(const char* Id) = 0;

	virtual void SetFillMode(bool Wireframe) = 0;
	virtual void SetDepthMode() = 0;

	static Renderer* CreateDX11Renderer(void* hWnd, const char* shader_path);
};