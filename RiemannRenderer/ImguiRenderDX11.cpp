#include "ImguiRenderDX11.h"

#if defined(_WIN32)

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "imgui.h"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <vector>
#include <windows.h>

#include <d3dcompiler.h>

namespace Riemann
{
	namespace
	{
		template <class T>
		void SafeRelease(T*& value)
		{
			if (value)
			{
				value->Release();
				value = nullptr;
			}
		}

		struct ImguiConstants
		{
			float ScreenSize[2];
			float Padding[2];
		};

		const float kPi = 3.14159265358979323846f;
		const float kScale = 1.0f / 8.0f;
		const float kTabStops[] = { 150.0f, 210.0f, 270.0f, 330.0f };

		unsigned int Max3(unsigned int a, unsigned int b, unsigned int c)
		{
			return std::max(a, std::max(b, c));
		}
	}

	struct ImguiRenderDX11::Glyph
	{
		float U0 = 0.0f;
		float V0 = 0.0f;
		float U1 = 0.0f;
		float V1 = 0.0f;
		float Width = 0.0f;
		float Height = 0.0f;
		float Advance = 0.0f;
	};

	struct ImguiRenderDX11::Vertex
	{
		float Pos[2];
		float Tex[2];
		unsigned int Color;
	};

	ImguiRenderDX11::ImguiRenderDX11()
		: m_Device(nullptr)
		, m_Context(nullptr)
		, m_VertexShader(nullptr)
		, m_PixelShader(nullptr)
		, m_InputLayout(nullptr)
		, m_VertexBuffer(nullptr)
		, m_ConstantBuffer(nullptr)
		, m_BlendState(nullptr)
		, m_RasterizerState(nullptr)
		, m_DepthStencilState(nullptr)
		, m_SamplerState(nullptr)
		, m_WhiteTextureView(nullptr)
		, m_FontTextureView(nullptr)
		, m_VertexCapacity(0)
		, m_Width(1)
		, m_Height(1)
		, m_FontHeight(16)
		, m_Glyphs(new Glyph[96])
	{
	}

	ImguiRenderDX11::~ImguiRenderDX11()
	{
		Shutdown();
		delete[] m_Glyphs;
	}

	bool ImguiRenderDX11::Init(ID3D11Device* device, ID3D11DeviceContext* context)
	{
		Shutdown();

		if (device == nullptr || context == nullptr)
		{
			return false;
		}

		m_Device = device;
		m_Context = context;
		m_Device->AddRef();
		m_Context->AddRef();

		return CreateShaders() && CreateStates() && CreateWhiteTexture() && CreateFontTexture();
	}

	void ImguiRenderDX11::Shutdown()
	{
		SafeRelease(m_FontTextureView);
		SafeRelease(m_WhiteTextureView);
		SafeRelease(m_SamplerState);
		SafeRelease(m_DepthStencilState);
		SafeRelease(m_RasterizerState);
		SafeRelease(m_BlendState);
		SafeRelease(m_ConstantBuffer);
		SafeRelease(m_VertexBuffer);
		SafeRelease(m_InputLayout);
		SafeRelease(m_PixelShader);
		SafeRelease(m_VertexShader);
		SafeRelease(m_Context);
		SafeRelease(m_Device);
		m_VertexCapacity = 0;
	}

	bool ImguiRenderDX11::CreateShaders()
	{
		static const char* shaderSource =
			"cbuffer ImguiConstants : register(b0)\n"
			"{\n"
			"    float2 ScreenSize;\n"
			"    float2 Padding;\n"
			"};\n"
			"Texture2D FontTexture : register(t0);\n"
			"SamplerState FontSampler : register(s0);\n"
			"struct VSInput\n"
			"{\n"
			"    float2 Position : POSITION;\n"
			"    float2 TexCoord : TEXCOORD0;\n"
			"    float4 Color : COLOR0;\n"
			"};\n"
			"struct VSOutput\n"
			"{\n"
			"    float4 Position : SV_POSITION;\n"
			"    float2 TexCoord : TEXCOORD0;\n"
			"    float4 Color : COLOR0;\n"
			"};\n"
			"VSOutput VS(VSInput input)\n"
			"{\n"
			"    VSOutput output;\n"
			"    float2 p = input.Position / ScreenSize;\n"
			"    output.Position = float4(p.x * 2.0f - 1.0f, 1.0f - p.y * 2.0f, 0.0f, 1.0f);\n"
			"    output.TexCoord = input.TexCoord;\n"
			"    output.Color = input.Color;\n"
			"    return output;\n"
			"}\n"
			"float4 PS(VSOutput input) : SV_TARGET\n"
			"{\n"
			"    return input.Color * FontTexture.Sample(FontSampler, input.TexCoord);\n"
			"}\n";

		ID3DBlob* vsBlob = nullptr;
		ID3DBlob* psBlob = nullptr;
		ID3DBlob* errorBlob = nullptr;

		HRESULT hr = D3DCompile(shaderSource, std::strlen(shaderSource), nullptr, nullptr, nullptr, "VS", "vs_4_0", 0, 0, &vsBlob, &errorBlob);
		if (FAILED(hr))
		{
			if (errorBlob)
			{
				OutputDebugStringA(reinterpret_cast<const char*>(errorBlob->GetBufferPointer()));
				errorBlob->Release();
			}
			return false;
		}
		SafeRelease(errorBlob);

		hr = D3DCompile(shaderSource, std::strlen(shaderSource), nullptr, nullptr, nullptr, "PS", "ps_4_0", 0, 0, &psBlob, &errorBlob);
		if (FAILED(hr))
		{
			if (errorBlob)
			{
				OutputDebugStringA(reinterpret_cast<const char*>(errorBlob->GetBufferPointer()));
				errorBlob->Release();
			}
			vsBlob->Release();
			return false;
		}
		SafeRelease(errorBlob);

		hr = m_Device->CreateVertexShader(vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), nullptr, &m_VertexShader);
		if (SUCCEEDED(hr))
		{
			hr = m_Device->CreatePixelShader(psBlob->GetBufferPointer(), psBlob->GetBufferSize(), nullptr, &m_PixelShader);
		}

		if (SUCCEEDED(hr))
		{
			D3D11_INPUT_ELEMENT_DESC layout[] =
			{
				{ "POSITION", 0, DXGI_FORMAT_R32G32_FLOAT, 0, offsetof(Vertex, Pos), D3D11_INPUT_PER_VERTEX_DATA, 0 },
				{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, offsetof(Vertex, Tex), D3D11_INPUT_PER_VERTEX_DATA, 0 },
				{ "COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, offsetof(Vertex, Color), D3D11_INPUT_PER_VERTEX_DATA, 0 },
			};
			hr = m_Device->CreateInputLayout(layout, ARRAYSIZE(layout), vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), &m_InputLayout);
		}

		vsBlob->Release();
		psBlob->Release();

		if (FAILED(hr))
		{
			return false;
		}

		D3D11_BUFFER_DESC cbDesc = {};
		cbDesc.ByteWidth = sizeof(ImguiConstants);
		cbDesc.Usage = D3D11_USAGE_DEFAULT;
		cbDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		return SUCCEEDED(m_Device->CreateBuffer(&cbDesc, nullptr, &m_ConstantBuffer));
	}

	bool ImguiRenderDX11::CreateStates()
	{
		D3D11_BLEND_DESC blendDesc = {};
		blendDesc.RenderTarget[0].BlendEnable = TRUE;
		blendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
		blendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
		blendDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
		blendDesc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
		blendDesc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
		blendDesc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
		blendDesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
		if (FAILED(m_Device->CreateBlendState(&blendDesc, &m_BlendState)))
		{
			return false;
		}

		D3D11_RASTERIZER_DESC rasterDesc = {};
		rasterDesc.FillMode = D3D11_FILL_SOLID;
		rasterDesc.CullMode = D3D11_CULL_NONE;
		rasterDesc.ScissorEnable = TRUE;
		rasterDesc.DepthClipEnable = TRUE;
		if (FAILED(m_Device->CreateRasterizerState(&rasterDesc, &m_RasterizerState)))
		{
			return false;
		}

		D3D11_DEPTH_STENCIL_DESC depthDesc = {};
		depthDesc.DepthEnable = FALSE;
		depthDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
		depthDesc.DepthFunc = D3D11_COMPARISON_ALWAYS;
		if (FAILED(m_Device->CreateDepthStencilState(&depthDesc, &m_DepthStencilState)))
		{
			return false;
		}

		D3D11_SAMPLER_DESC samplerDesc = {};
		samplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
		samplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
		samplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
		samplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
		samplerDesc.ComparisonFunc = D3D11_COMPARISON_ALWAYS;
		samplerDesc.MaxLOD = D3D11_FLOAT32_MAX;
		return SUCCEEDED(m_Device->CreateSamplerState(&samplerDesc, &m_SamplerState));
	}

	bool ImguiRenderDX11::CreateWhiteTexture()
	{
		const unsigned int whitePixel = 0xffffffffu;

		D3D11_TEXTURE2D_DESC desc = {};
		desc.Width = 1;
		desc.Height = 1;
		desc.MipLevels = 1;
		desc.ArraySize = 1;
		desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		desc.SampleDesc.Count = 1;
		desc.Usage = D3D11_USAGE_DEFAULT;
		desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;

		D3D11_SUBRESOURCE_DATA data = {};
		data.pSysMem = &whitePixel;
		data.SysMemPitch = sizeof(whitePixel);

		ID3D11Texture2D* texture = nullptr;
		HRESULT hr = m_Device->CreateTexture2D(&desc, &data, &texture);
		if (FAILED(hr))
		{
			return false;
		}

		hr = m_Device->CreateShaderResourceView(texture, nullptr, &m_WhiteTextureView);
		texture->Release();
		return SUCCEEDED(hr);
	}

	bool ImguiRenderDX11::CreateFontTexture()
	{
		const int atlasWidth = 512;
		const int atlasHeight = 512;

		BITMAPINFO bitmapInfo = {};
		bitmapInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
		bitmapInfo.bmiHeader.biWidth = atlasWidth;
		bitmapInfo.bmiHeader.biHeight = -atlasHeight;
		bitmapInfo.bmiHeader.biPlanes = 1;
		bitmapInfo.bmiHeader.biBitCount = 32;
		bitmapInfo.bmiHeader.biCompression = BI_RGB;

		void* dibPixels = nullptr;
		HDC dc = CreateCompatibleDC(nullptr);
		if (dc == nullptr)
		{
			return false;
		}

		HBITMAP bitmap = CreateDIBSection(dc, &bitmapInfo, DIB_RGB_COLORS, &dibPixels, nullptr, 0);
		if (bitmap == nullptr || dibPixels == nullptr)
		{
			DeleteDC(dc);
			return false;
		}

		HGDIOBJ oldBitmap = SelectObject(dc, bitmap);
		std::memset(dibPixels, 0, atlasWidth * atlasHeight * 4);

		HFONT font = CreateFontA(
			-15, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
			ANSI_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
			ANTIALIASED_QUALITY, DEFAULT_PITCH | FF_DONTCARE, "Segoe UI");
		HGDIOBJ oldFont = nullptr;
		if (font)
		{
			oldFont = SelectObject(dc, font);
		}

		SetBkMode(dc, TRANSPARENT);
		SetTextColor(dc, RGB(255, 255, 255));

		TEXTMETRICA metrics = {};
		GetTextMetricsA(dc, &metrics);
		m_FontHeight = std::max(8, static_cast<int>(metrics.tmHeight));

		int penX = 1;
		int penY = 1;
		int rowHeight = m_FontHeight + 2;

		for (int c = 32; c < 128; ++c)
		{
			char ch = static_cast<char>(c);
			SIZE size = {};
			GetTextExtentPoint32A(dc, &ch, 1, &size);
			const int glyphWidth = std::max(1L, size.cx);
			const int glyphHeight = m_FontHeight;

			if (penX + glyphWidth + 2 >= atlasWidth)
			{
				penX = 1;
				penY += rowHeight;
			}

			if (penY + glyphHeight + 2 >= atlasHeight)
			{
				break;
			}

			Glyph& glyph = m_Glyphs[c - 32];
			glyph.U0 = penX / static_cast<float>(atlasWidth);
			glyph.V0 = penY / static_cast<float>(atlasHeight);
			glyph.U1 = (penX + glyphWidth) / static_cast<float>(atlasWidth);
			glyph.V1 = (penY + glyphHeight) / static_cast<float>(atlasHeight);
			glyph.Width = static_cast<float>(glyphWidth);
			glyph.Height = static_cast<float>(glyphHeight);
			glyph.Advance = static_cast<float>(std::max(1L, size.cx));

			TextOutA(dc, penX, penY, &ch, 1);
			penX += glyphWidth + 1;
		}

		std::vector<unsigned char> rgba(atlasWidth * atlasHeight * 4);
		const unsigned char* source = static_cast<const unsigned char*>(dibPixels);
		for (int i = 0; i < atlasWidth * atlasHeight; ++i)
		{
			const unsigned int alpha = Max3(source[i * 4 + 0], source[i * 4 + 1], source[i * 4 + 2]);
			rgba[i * 4 + 0] = 255;
			rgba[i * 4 + 1] = 255;
			rgba[i * 4 + 2] = 255;
			rgba[i * 4 + 3] = static_cast<unsigned char>(alpha);
		}

		if (oldFont)
		{
			SelectObject(dc, oldFont);
		}
		if (font)
		{
			DeleteObject(font);
		}
		SelectObject(dc, oldBitmap);
		DeleteObject(bitmap);
		DeleteDC(dc);

		D3D11_TEXTURE2D_DESC desc = {};
		desc.Width = atlasWidth;
		desc.Height = atlasHeight;
		desc.MipLevels = 1;
		desc.ArraySize = 1;
		desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		desc.SampleDesc.Count = 1;
		desc.Usage = D3D11_USAGE_DEFAULT;
		desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;

		D3D11_SUBRESOURCE_DATA data = {};
		data.pSysMem = rgba.data();
		data.SysMemPitch = atlasWidth * 4;

		ID3D11Texture2D* texture = nullptr;
		HRESULT hr = m_Device->CreateTexture2D(&desc, &data, &texture);
		if (FAILED(hr))
		{
			return false;
		}

		hr = m_Device->CreateShaderResourceView(texture, nullptr, &m_FontTextureView);
		texture->Release();
		return SUCCEEDED(hr);
	}

	bool ImguiRenderDX11::EnsureVertexBuffer(unsigned int vertexCount)
	{
		if (vertexCount == 0)
		{
			return true;
		}

		if (m_VertexBuffer && m_VertexCapacity >= vertexCount)
		{
			return true;
		}

		SafeRelease(m_VertexBuffer);
		m_VertexCapacity = std::max(256u, vertexCount + 128u);

		D3D11_BUFFER_DESC desc = {};
		desc.Usage = D3D11_USAGE_DYNAMIC;
		desc.ByteWidth = sizeof(Vertex) * m_VertexCapacity;
		desc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		return SUCCEEDED(m_Device->CreateBuffer(&desc, nullptr, &m_VertexBuffer));
	}

	void ImguiRenderDX11::Draw(int width, int height)
	{
		if (m_Context == nullptr || m_VertexShader == nullptr || m_PixelShader == nullptr)
		{
			return;
		}

		m_Width = std::max(1, width);
		m_Height = std::max(1, height);

		D3D11_VIEWPORT viewport = {};
		viewport.Width = static_cast<float>(m_Width);
		viewport.Height = static_cast<float>(m_Height);
		viewport.MinDepth = 0.0f;
		viewport.MaxDepth = 1.0f;
		m_Context->RSSetViewports(1, &viewport);

		const float blendFactor[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
		m_Context->OMSetBlendState(m_BlendState, blendFactor, 0xffffffffu);
		m_Context->OMSetDepthStencilState(m_DepthStencilState, 0);
		m_Context->RSSetState(m_RasterizerState);

		m_Context->IASetInputLayout(m_InputLayout);
		m_Context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
		m_Context->VSSetShader(m_VertexShader, nullptr, 0);
		m_Context->PSSetShader(m_PixelShader, nullptr, 0);
		m_Context->PSSetSamplers(0, 1, &m_SamplerState);

		ImguiConstants constants = {};
		constants.ScreenSize[0] = static_cast<float>(m_Width);
		constants.ScreenSize[1] = static_cast<float>(m_Height);
		m_Context->UpdateSubresource(m_ConstantBuffer, 0, nullptr, &constants, 0, 0);
		m_Context->VSSetConstantBuffers(0, 1, &m_ConstantBuffer);

		SetScissorRect(0, 0, m_Width, m_Height);
		DrawRenderQueue();
		SetScissorRect(0, 0, m_Width, m_Height);
	}

	void ImguiRenderDX11::DrawRenderQueue()
	{
		const imguiGfxCmd* commands = imguiGetRenderQueue();
		const int commandCount = imguiGetRenderQueueSize();

		for (int i = 0; i < commandCount; ++i)
		{
			const imguiGfxCmd& cmd = commands[i];
			if (cmd.type == IMGUI_GFXCMD_RECT)
			{
				const float x = static_cast<float>(cmd.rect.x) * kScale + 0.5f;
				const float y = static_cast<float>(cmd.rect.y) * kScale + 0.5f;
				const float w = static_cast<float>(cmd.rect.w) * kScale - 1.0f;
				const float h = static_cast<float>(cmd.rect.h) * kScale - 1.0f;
				const float r = static_cast<float>(cmd.rect.r) * kScale;
				if (cmd.rect.r == 0)
				{
					AddRect(x, y, w, h, cmd.col);
				}
				else
				{
					AddRoundedRect(x, y, w, h, r, cmd.col);
				}
			}
			else if (cmd.type == IMGUI_GFXCMD_LINE)
			{
				AddLine(
					static_cast<float>(cmd.line.x0) * kScale,
					static_cast<float>(cmd.line.y0) * kScale,
					static_cast<float>(cmd.line.x1) * kScale,
					static_cast<float>(cmd.line.y1) * kScale,
					static_cast<float>(cmd.line.r) * kScale,
					cmd.col);
			}
			else if (cmd.type == IMGUI_GFXCMD_TRIANGLE)
			{
				AddTriangle(
					static_cast<float>(cmd.rect.x) * kScale + 0.5f,
					static_cast<float>(cmd.rect.y) * kScale + 0.5f,
					static_cast<float>(cmd.rect.w) * kScale - 1.0f,
					static_cast<float>(cmd.rect.h) * kScale - 1.0f,
					cmd.flags,
					cmd.col);
			}
			else if (cmd.type == IMGUI_GFXCMD_TEXT)
			{
				AddText(static_cast<float>(cmd.text.x), static_cast<float>(cmd.text.y), cmd.text.align, cmd.text.text, cmd.col);
			}
			else if (cmd.type == IMGUI_GFXCMD_SCISSOR)
			{
				if (cmd.flags)
				{
					SetScissorRect(cmd.rect.x, cmd.rect.y, cmd.rect.w, cmd.rect.h);
				}
				else
				{
					SetScissorRect(0, 0, m_Width, m_Height);
				}
			}
		}
	}

	void ImguiRenderDX11::DrawCommandGeometry(const Vertex* vertices, unsigned int vertexCount, ID3D11ShaderResourceView* texture)
	{
		if (vertices == nullptr || vertexCount == 0 || texture == nullptr)
		{
			return;
		}

		if (!EnsureVertexBuffer(vertexCount))
		{
			return;
		}

		D3D11_MAPPED_SUBRESOURCE mapped = {};
		if (FAILED(m_Context->Map(m_VertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)))
		{
			return;
		}
		std::memcpy(mapped.pData, vertices, sizeof(Vertex) * vertexCount);
		m_Context->Unmap(m_VertexBuffer, 0);

		const UINT stride = sizeof(Vertex);
		const UINT offset = 0;
		m_Context->IASetVertexBuffers(0, 1, &m_VertexBuffer, &stride, &offset);
		m_Context->PSSetShaderResources(0, 1, &texture);
		m_Context->Draw(vertexCount, 0);
	}

	void ImguiRenderDX11::SetScissorRect(int x, int y, int w, int h)
	{
		RECT rect = {};
		rect.left = std::max(0, x);
		rect.top = std::max(0, m_Height - (y + h));
		rect.right = std::min(m_Width, x + w);
		rect.bottom = std::min(m_Height, m_Height - y);
		if (rect.right < rect.left)
		{
			rect.right = rect.left;
		}
		if (rect.bottom < rect.top)
		{
			rect.bottom = rect.top;
		}
		m_Context->RSSetScissorRects(1, &rect);
	}

	float ImguiRenderDX11::ToScreenY(float y) const
	{
		return static_cast<float>(m_Height) - y;
	}

	void ImguiRenderDX11::AddRect(float x, float y, float w, float h, unsigned int color)
	{
		const float left = x;
		const float right = x + w;
		const float top = ToScreenY(y + h);
		const float bottom = ToScreenY(y);

		Vertex vertices[6] =
		{
			{ { left, top }, { 0.0f, 0.0f }, color },
			{ { right, top }, { 0.0f, 0.0f }, color },
			{ { right, bottom }, { 0.0f, 0.0f }, color },
			{ { left, top }, { 0.0f, 0.0f }, color },
			{ { right, bottom }, { 0.0f, 0.0f }, color },
			{ { left, bottom }, { 0.0f, 0.0f }, color },
		};
		DrawCommandGeometry(vertices, 6, m_WhiteTextureView);
	}

	void ImguiRenderDX11::AddRoundedRect(float x, float y, float w, float h, float radius, unsigned int color)
	{
		radius = std::max(0.0f, std::min(radius, std::min(w, h) * 0.5f));
		if (radius <= 0.5f)
		{
			AddRect(x, y, w, h, color);
			return;
		}

		const float left = x;
		const float right = x + w;
		const float top = ToScreenY(y + h);
		const float bottom = ToScreenY(y);
		const float r = radius;

		std::vector<Vertex> fan;
		fan.reserve(6 * 4 * 3);
		const float centerX = (left + right) * 0.5f;
		const float centerY = (top + bottom) * 0.5f;

		std::vector<Vertex> points;
		points.reserve(28);
		const int segments = 6;
		const float centers[4][2] =
		{
			{ right - r, top + r },
			{ right - r, bottom - r },
			{ left + r, bottom - r },
			{ left + r, top + r },
		};
		const float startAngles[4] =
		{
			-kPi * 0.5f,
			0.0f,
			kPi * 0.5f,
			kPi,
		};

		for (int corner = 0; corner < 4; ++corner)
		{
			for (int i = 0; i <= segments; ++i)
			{
				const float angle = startAngles[corner] + (kPi * 0.5f) * (static_cast<float>(i) / static_cast<float>(segments));
				Vertex v = {};
				v.Pos[0] = centers[corner][0] + std::cos(angle) * r;
				v.Pos[1] = centers[corner][1] + std::sin(angle) * r;
				v.Color = color;
				points.push_back(v);
			}
		}

		Vertex center = {};
		center.Pos[0] = centerX;
		center.Pos[1] = centerY;
		center.Color = color;
		for (size_t i = 0; i < points.size(); ++i)
		{
			fan.push_back(center);
			fan.push_back(points[i]);
			fan.push_back(points[(i + 1) % points.size()]);
		}
		DrawCommandGeometry(fan.data(), static_cast<unsigned int>(fan.size()), m_WhiteTextureView);
	}

	void ImguiRenderDX11::AddLine(float x0, float y0, float x1, float y1, float radius, unsigned int color)
	{
		y0 = ToScreenY(y0);
		y1 = ToScreenY(y1);

		float dx = x1 - x0;
		float dy = y1 - y0;
		const float len = std::sqrt(dx * dx + dy * dy);
		if (len <= 0.0001f)
		{
			return;
		}
		dx /= len;
		dy /= len;

		const float halfWidth = std::max(1.0f, radius) * 0.5f;
		const float nx = -dy * halfWidth;
		const float ny = dx * halfWidth;

		Vertex vertices[6] =
		{
			{ { x0 - nx, y0 - ny }, { 0.0f, 0.0f }, color },
			{ { x0 + nx, y0 + ny }, { 0.0f, 0.0f }, color },
			{ { x1 + nx, y1 + ny }, { 0.0f, 0.0f }, color },
			{ { x0 - nx, y0 - ny }, { 0.0f, 0.0f }, color },
			{ { x1 + nx, y1 + ny }, { 0.0f, 0.0f }, color },
			{ { x1 - nx, y1 - ny }, { 0.0f, 0.0f }, color },
		};
		DrawCommandGeometry(vertices, 6, m_WhiteTextureView);
	}

	void ImguiRenderDX11::AddTriangle(float x, float y, float w, float h, int flags, unsigned int color)
	{
		Vertex vertices[3] = {};
		if (flags == 1)
		{
			vertices[0].Pos[0] = x;
			vertices[0].Pos[1] = ToScreenY(y);
			vertices[1].Pos[0] = x + w;
			vertices[1].Pos[1] = ToScreenY(y + h * 0.5f);
			vertices[2].Pos[0] = x;
			vertices[2].Pos[1] = ToScreenY(y + h);
		}
		else
		{
			vertices[0].Pos[0] = x;
			vertices[0].Pos[1] = ToScreenY(y + h);
			vertices[1].Pos[0] = x + w * 0.5f;
			vertices[1].Pos[1] = ToScreenY(y);
			vertices[2].Pos[0] = x + w;
			vertices[2].Pos[1] = ToScreenY(y + h);
		}
		vertices[0].Color = color;
		vertices[1].Color = color;
		vertices[2].Color = color;
		DrawCommandGeometry(vertices, 3, m_WhiteTextureView);
	}

	float ImguiRenderDX11::MeasureText(const char* text) const
	{
		if (text == nullptr)
		{
			return 0.0f;
		}

		float x = 0.0f;
		float length = 0.0f;
		while (*text)
		{
			const unsigned char c = static_cast<unsigned char>(*text);
			if (c == '\t')
			{
				for (float tabStop : kTabStops)
				{
					if (x < tabStop)
					{
						x = tabStop;
						break;
					}
				}
				length = std::max(length, x);
			}
			else if (c >= 32 && c < 128)
			{
				const Glyph& glyph = m_Glyphs[c - 32];
				length = std::max(length, x + glyph.Width);
				x += glyph.Advance;
			}
			++text;
		}
		return length;
	}

	void ImguiRenderDX11::AddText(float x, float y, int align, const char* text, unsigned int color)
	{
		if (text == nullptr || m_FontTextureView == nullptr)
		{
			return;
		}

		if (align == IMGUI_ALIGN_CENTER)
		{
			x -= MeasureText(text) * 0.5f;
		}
		else if (align == IMGUI_ALIGN_RIGHT)
		{
			x -= MeasureText(text);
		}

		const float originX = x;
		const float top = ToScreenY(y + static_cast<float>(m_FontHeight));
		std::vector<Vertex> vertices;
		vertices.reserve(std::strlen(text) * 6);

		while (*text)
		{
			const unsigned char c = static_cast<unsigned char>(*text);
			if (c == '\t')
			{
				for (float tabStop : kTabStops)
				{
					if (x < originX + tabStop)
					{
						x = originX + tabStop;
						break;
					}
				}
			}
			else if (c >= 32 && c < 128)
			{
				const Glyph& glyph = m_Glyphs[c - 32];
				const float left = x;
				const float right = x + glyph.Width;
				const float bottom = top + glyph.Height;

				vertices.push_back({ { left, top }, { glyph.U0, glyph.V0 }, color });
				vertices.push_back({ { right, top }, { glyph.U1, glyph.V0 }, color });
				vertices.push_back({ { right, bottom }, { glyph.U1, glyph.V1 }, color });
				vertices.push_back({ { left, top }, { glyph.U0, glyph.V0 }, color });
				vertices.push_back({ { right, bottom }, { glyph.U1, glyph.V1 }, color });
				vertices.push_back({ { left, bottom }, { glyph.U0, glyph.V1 }, color });

				x += glyph.Advance;
			}
			++text;
		}

		if (!vertices.empty())
		{
			DrawCommandGeometry(vertices.data(), static_cast<unsigned int>(vertices.size()), m_FontTextureView);
		}
	}
}

#endif
