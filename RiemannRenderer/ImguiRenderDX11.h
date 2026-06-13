#pragma once

#if defined(_WIN32)

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <d3d11.h>

namespace Riemann
{
	class ImguiRenderDX11
	{
	public:
		ImguiRenderDX11();
		ImguiRenderDX11(const ImguiRenderDX11&) = delete;
		ImguiRenderDX11& operator=(const ImguiRenderDX11&) = delete;
		~ImguiRenderDX11();

		bool Init(ID3D11Device* device, ID3D11DeviceContext* context);
		void Shutdown();
		void Draw(int width, int height);

	private:
		struct Glyph;
		struct Vertex;

		bool CreateShaders();
		bool CreateStates();
		bool CreateWhiteTexture();
		bool CreateFontTexture();
		bool EnsureVertexBuffer(unsigned int vertexCount);

		void DrawRenderQueue();
		void DrawCommandGeometry(const Vertex* vertices, unsigned int vertexCount, ID3D11ShaderResourceView* texture);
		void SetScissorRect(int x, int y, int w, int h);

		void AddRect(float x, float y, float w, float h, unsigned int color);
		void AddRoundedRect(float x, float y, float w, float h, float radius, unsigned int color);
		void AddLine(float x0, float y0, float x1, float y1, float radius, unsigned int color);
		void AddTriangle(float x, float y, float w, float h, int flags, unsigned int color);
		void AddText(float x, float y, int align, const char* text, unsigned int color);

		float MeasureText(const char* text) const;
		float ToScreenY(float y) const;

	private:
		ID3D11Device* m_Device;
		ID3D11DeviceContext* m_Context;
		ID3D11VertexShader* m_VertexShader;
		ID3D11PixelShader* m_PixelShader;
		ID3D11InputLayout* m_InputLayout;
		ID3D11Buffer* m_VertexBuffer;
		ID3D11Buffer* m_ConstantBuffer;
		ID3D11BlendState* m_BlendState;
		ID3D11RasterizerState* m_RasterizerState;
		ID3D11DepthStencilState* m_DepthStencilState;
		ID3D11SamplerState* m_SamplerState;
		ID3D11ShaderResourceView* m_WhiteTextureView;
		ID3D11ShaderResourceView* m_FontTextureView;
		unsigned int m_VertexCapacity;
		int m_Width;
		int m_Height;
		int m_FontHeight;
		Glyph* m_Glyphs;
	};
}

#endif
