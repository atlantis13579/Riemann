#pragma once

#include "RiemannRenderer.h"

namespace Riemann
{
	class MetalRenderer final : public Renderer
	{
	public:
		MetalRenderer();
		~MetalRenderer() override;

		bool Init(const RendererCreateInfo& createInfo);

		void Render() override;
		void SetCamera(const CameraDesc& camera) override;
		void SetLight(const DirectionalLightDesc& light) override;
		bool AddMesh(const RenderMeshDesc& mesh) override;
		bool UpdateTransform(const RenderTransformUpdate& transform) override;
		bool UpdateVertices(const char* id, const Vertex1* vertices, int vertexCount) override;
		bool DeleteMesh(const char* id) override;
		bool Reset() override;

		void SetFillMode(bool wireframe) override;
		void SetDepthMode() override;
		void SetImguiDrawCallback(ImguiDrawCallback callback, void* userData) override;
		void UpdateImguiInput(const ImguiInputState& input) override;

	private:
		struct Impl;
		Impl* m_Impl;
	};
}
