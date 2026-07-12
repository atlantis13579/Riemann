#include "RiemannRenderer.h"

#include <algorithm>
#include <map>

namespace Riemann
{
	namespace
	{
		class NullRenderer final : public Renderer
		{
		public:
			void Render() override {}

			void SetCamera(const CameraDesc& camera) override
			{
				m_Camera = camera;
			}

			void SetLight(const DirectionalLightDesc& light) override
			{
				m_Light = light;
			}

			bool AddMesh(const RenderMeshDesc& mesh) override
			{
				if (mesh.Id.empty() || mesh.Vertices.empty() != mesh.Indices.empty()) return false;
				if ((mesh.Topology == RenderPrimitiveTopology::Triangles && mesh.Indices.size() % 3 != 0)
					|| (mesh.Topology == RenderPrimitiveTopology::Lines && mesh.Indices.size() % 2 != 0))
				{
					return false;
				}
				for (uint32_t index : mesh.Indices)
				{
					if (index >= mesh.Vertices.size()) return false;
				}
				m_Meshes[mesh.Id] = mesh;
				return true;
			}

			bool UpdateTransform(const RenderTransformUpdate& transform) override
			{
				auto it = m_Meshes.find(transform.Id);
				if (it == m_Meshes.end())
				{
					return false;
				}

				it->second.WorldTransform = transform.WorldTransform;
				return true;
			}

			bool UpdateVertices(const char* id, const Vertex1* vertices, int vertexCount) override
			{
				if (id == nullptr || vertices == nullptr || vertexCount <= 0)
				{
					return false;
				}

				auto it = m_Meshes.find(id);
				if (it == m_Meshes.end())
				{
					return false;
				}

				if ((size_t)vertexCount != it->second.Vertices.size()) return false;
				it->second.Vertices.assign(vertices, vertices + vertexCount);
				return true;
			}

			bool UpdateMeshGeometry(const char* id, const Vertex1* vertices, int vertexCount,
				const uint32_t* indices, int indexCount) override
			{
				if (id == nullptr || vertexCount < 0 || indexCount < 0
					|| ((vertexCount == 0 || indexCount == 0) && (vertexCount != 0 || indexCount != 0))
					|| (vertexCount > 0 && (vertices == nullptr || indices == nullptr)))
				{
					return false;
				}
				auto it = m_Meshes.find(id);
				if (it == m_Meshes.end()) return false;
				if ((it->second.Topology == RenderPrimitiveTopology::Triangles && indexCount % 3 != 0)
					|| (it->second.Topology == RenderPrimitiveTopology::Lines && indexCount % 2 != 0))
				{
					return false;
				}
				for (int index = 0; index < indexCount; ++index)
				{
					if (indices[index] >= (uint32_t)vertexCount) return false;
				}
				if (vertexCount == 0)
				{
					it->second.Vertices.clear();
					it->second.Indices.clear();
				}
				else
				{
					it->second.Vertices.assign(vertices, vertices + vertexCount);
					it->second.Indices.assign(indices, indices + indexCount);
				}
				return true;
			}

			bool DeleteMesh(const char* id) override
			{
				return id != nullptr && m_Meshes.erase(id) != 0;
			}

			bool Reset() override
			{
				m_Meshes.clear();
				return true;
			}

			void SetFillMode(bool wireframe) override
			{
				m_Wireframe = wireframe;
			}

			void SetDepthMode() override {}

		private:
			CameraDesc m_Camera;
			DirectionalLightDesc m_Light;
			std::map<std::string, RenderMeshDesc> m_Meshes;
			bool m_Wireframe = false;
		};
	}

	Renderer* Renderer::CreatePlatformRenderer(const RendererCreateInfo& createInfo)
	{
#if defined(_WIN32)
		return CreateDX11Renderer(createInfo.NativeWindow, createInfo.ShaderPath.c_str());
#elif defined(__APPLE__)
		return CreateMetalRenderer(createInfo);
#else
		(void)createInfo;
		return CreateNullRenderer();
#endif
	}

	Renderer* Renderer::CreateNullRenderer()
	{
		return new NullRenderer();
	}

#if !defined(__APPLE__)
	Renderer* Renderer::CreateMetalRenderer(const RendererCreateInfo& createInfo)
	{
		(void)createInfo;
		return nullptr;
	}
#endif

#if !defined(_WIN32)
	Renderer* Renderer::CreateDX11Renderer(void* hWnd, const char* shaderPath)
	{
		(void)hWnd;
		(void)shaderPath;
		return nullptr;
	}
#endif
}
