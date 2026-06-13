#pragma once

#include <stdint.h>
#include <string>
#include <vector>

#include "../Src/Maths/Transform.h"
#include "../Src/Maths/Vector3.h"
#include "../Src/Maths/Vector4.h"

#if defined(_WIN32) && !defined(RENDERER_STATIC)
#  ifdef RENDERER_EXPORT
#    define RENDERER_API __declspec(dllexport)
#  else
#    define RENDERER_API __declspec(dllimport)
#  endif
#else
#  define RENDERER_API
#endif

struct Vertex1
{
	Vertex1() {}
	Vertex1(const Vector3& p, const Vector3& n)
		: Pos(p)
		, Normal(n)
	{
	}

	Vector3 Pos;
	Vector3 Normal;
};

namespace Riemann
{
	enum class RenderPrimitiveTopology : uint8_t
	{
		Triangles,
		Lines,
	};

	struct RenderMeshDesc
	{
		std::string Id;
		std::vector<Vertex1> Vertices;
		std::vector<uint32_t> Indices;
		Transform WorldTransform;
		Vector4 Color = Vector4(0.72f, 0.76f, 0.82f, 1.0f);
		RenderPrimitiveTopology Topology = RenderPrimitiveTopology::Triangles;
		bool CastShadow = true;
	};

	struct RenderTransformUpdate
	{
		std::string Id;
		Transform WorldTransform;
	};

	struct CameraDesc
	{
		Vector3 Eye = Vector3(0.0f, 0.0f, 5.0f);
		Vector3 At = Vector3::Zero();
		Vector3 Up = Vector3::UnitY();
		float FovY = 1.57079632679f;
		float NearPlane = 0.001f;
		float FarPlane = 100000.0f;
	};

	struct DirectionalLightDesc
	{
		Vector3 Direction = Vector3(-0.4f, -1.0f, 0.35f);
		Vector3 Color = Vector3(1.0f, 0.96f, 0.86f);
		Vector3 ShadowCenter = Vector3::Zero();
		float Ambient = 0.22f;
		float ShadowDistance = 60.0f;
		float ShadowSize = 80.0f;
	};

	struct RendererCreateInfo
	{
		// Windows: HWND. macOS: NSView*, NSWindow*, or CAMetalLayer*.
		void* NativeWindow = nullptr;
		std::string ShaderPath;
		int Width = 1024;
		int Height = 768;
	};

	class RENDERER_API Renderer
	{
	public:
		Renderer() {}
		virtual ~Renderer() {}

		virtual void Render() = 0;
		virtual void SetCamera(const CameraDesc& camera) = 0;
		virtual void SetLight(const DirectionalLightDesc& light) = 0;
		virtual bool AddMesh(const RenderMeshDesc& mesh) = 0;
		virtual bool UpdateTransform(const RenderTransformUpdate& transform) = 0;
		virtual bool UpdateVertices(const char* id, const Vertex1* vertices, int vertexCount) = 0;
		virtual bool DeleteMesh(const char* id) = 0;
		virtual bool Reset() = 0;

		virtual void SetFillMode(bool wireframe) = 0;
		virtual void SetDepthMode() = 0;

		static Renderer* CreatePlatformRenderer(const RendererCreateInfo& createInfo);
		static Renderer* CreateDX11Renderer(void* hWnd, const char* shaderPath);
		static Renderer* CreateMetalRenderer(const RendererCreateInfo& createInfo);
		static Renderer* CreateNullRenderer();
	};
}
