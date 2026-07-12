#include "DX11Renderer.h"

#if defined(_WIN32)

#include <assert.h>
#include <algorithm>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <windows.h>
#undef min
#undef max

#include <d3d11_1.h>
#include <d3dcompiler.h>
#include <directxcolors.h>

#include "../Src/Maths/Matrix4.h"
#include "../Src/Maths/Quaternion.h"
#include "../Src/Maths/Transform.h"
#include "../Src/Maths/Vector4.h"
#include "imgui.h"
#include "ImguiRenderDX11.h"

using namespace DirectX;

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

	Matrix4 GetTransformMatrix(const Transform& transform)
	{
		return Transform3::Compose(transform.pos, transform.quat);
	}

	Vector3 SafeUnit(const Vector3& value, const Vector3& fallback)
	{
		const float lenSq = value.SquareLength();
		if (lenSq <= 1e-8f)
		{
			return fallback;
		}
		return value / sqrtf(lenSq);
	}

	std::wstring ToWidePath(const char* path)
	{
		std::wstring result;
		if (path == nullptr)
		{
			return result;
		}

		while (*path != '\0')
		{
			result.push_back(static_cast<wchar_t>(*path));
			++path;
		}
		return result;
	}

	struct ConstantBuffer
	{
		Matrix4 World;
		Matrix4 View;
		Matrix4 Projection;
		Matrix4 LightView;
		Matrix4 LightProjection;
		Vector4 EyePos;
		Vector4 LightDir;
		Vector4 LightColor;
		Vector4 MaterialColor;
		Vector4 OutlineColor;
		Vector4 RenderParams;
	};

	struct DX11StaticMesh
	{
		DX11StaticMesh() {}
		DX11StaticMesh(const DX11StaticMesh&) = delete;
		DX11StaticMesh& operator=(const DX11StaticMesh&) = delete;

		DX11StaticMesh(DX11StaticMesh&& rhs) noexcept
		{
			MoveFrom(rhs);
		}

		DX11StaticMesh& operator=(DX11StaticMesh&& rhs) noexcept
		{
			if (this != &rhs)
			{
				Release();
				MoveFrom(rhs);
			}
			return *this;
		}

		~DX11StaticMesh()
		{
			Release();
		}

		void Release()
		{
			SafeRelease(pConstantBuffer);
			SafeRelease(pVertexBuffer);
			SafeRelease(pIndexBuffer);
		}

		void MoveFrom(DX11StaticMesh& rhs)
		{
			Id = std::move(rhs.Id);
			pVertexBuffer = rhs.pVertexBuffer;
			pIndexBuffer = rhs.pIndexBuffer;
			pConstantBuffer = rhs.pConstantBuffer;
			WorldTransform = rhs.WorldTransform;
			Color = rhs.Color;
			VertexCount = rhs.VertexCount;
			IndexCount = rhs.IndexCount;
			VertexCapacity = rhs.VertexCapacity;
			IndexCapacity = rhs.IndexCapacity;
			IndexFormat = rhs.IndexFormat;
			Topology = rhs.Topology;
			FillMode = rhs.FillMode;
			BlendMode = rhs.BlendMode;
			CastShadow = rhs.CastShadow;
			Unlit = rhs.Unlit;
			OutlineEnabled = rhs.OutlineEnabled;
			OutlineColor = rhs.OutlineColor;
			OutlineThickness = rhs.OutlineThickness;

			rhs.pVertexBuffer = nullptr;
			rhs.pIndexBuffer = nullptr;
			rhs.pConstantBuffer = nullptr;
			rhs.VertexCount = 0;
			rhs.IndexCount = 0;
			rhs.VertexCapacity = 0;
			rhs.IndexCapacity = 0;
		}

		std::string Id;
		ID3D11Buffer* pVertexBuffer = nullptr;
		ID3D11Buffer* pIndexBuffer = nullptr;
		ID3D11Buffer* pConstantBuffer = nullptr;
		Transform WorldTransform;
		Vector4 Color = Vector4(0.72f, 0.76f, 0.82f, 1.0f);
		int VertexCount = 0;
		int IndexCount = 0;
		int VertexCapacity = 0;
		int IndexCapacity = 0;
		DXGI_FORMAT IndexFormat = DXGI_FORMAT_R32_UINT;
		D3D11_PRIMITIVE_TOPOLOGY Topology = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		Riemann::RenderFillMode FillMode = Riemann::RenderFillMode::RendererDefault;
		Riemann::RenderBlendMode BlendMode = Riemann::RenderBlendMode::Alpha;
		bool CastShadow = true;
		bool Unlit = false;
		bool OutlineEnabled = false;
		Vector4 OutlineColor = Vector4(1.0f, 0.82f, 0.18f, 1.0f);
		float OutlineThickness = 0.06f;
	};
}

class DX11Renderer final : public Riemann::Renderer
{
public:
	DX11Renderer() {}

	~DX11Renderer() override
	{
		m_ImguiRenderer.Shutdown();
		m_AllMesh.clear();

		if (m_pImmediateContext)
		{
			m_pImmediateContext->ClearState();
		}

		SafeRelease(m_pShadowSampler);
		SafeRelease(m_pShadowSRV);
		SafeRelease(m_pShadowDepthView);
		SafeRelease(m_pShadowTexture);
		SafeRelease(m_pAdditiveBlendState);
		SafeRelease(m_pTransparentBlendState);
		SafeRelease(m_pTransparentDepthStencilState);
		SafeRelease(m_pColorWriteOffBlendState);
		SafeRelease(m_pOutlineMaskDepthStencilState);
		SafeRelease(m_pOutlineDepthStencilState);
		SafeRelease(m_pDepthStencilState);
		SafeRelease(m_pOutlineRasterizerState);
		SafeRelease(m_pWireframeRasterizerState);
		SafeRelease(m_pSolidRasterizerState);
		SafeRelease(m_pVertexLayout);
		SafeRelease(m_pOutlinePixelShader);
		SafeRelease(m_pShadowVertexShader);
		SafeRelease(m_pOutlineVertexShader);
		SafeRelease(m_pVertexShader);
		SafeRelease(m_pPixelShader);
		SafeRelease(m_pDepthBuffer);
		SafeRelease(m_pRenderTargetView);
		SafeRelease(m_pSwapChain1);
		SafeRelease(m_pSwapChain);
		SafeRelease(m_pImmediateContext1);
		SafeRelease(m_pImmediateContext);
		SafeRelease(m_pd3dDevice1);
		SafeRelease(m_pd3dDevice);
	}

	HRESULT InitDevice(HWND hWnd, const char* shaderPath)
	{
		HRESULT hr = S_OK;
		m_hWnd = hWnd;

		RECT rc;
		GetClientRect(hWnd, &rc);
		m_Width = rc.right - rc.left;
		m_Height = rc.bottom - rc.top;
		if (m_Width == 0 || m_Height == 0)
		{
			return E_FAIL;
		}

		UINT createDeviceFlags = 0;
#ifdef _DEBUG
		createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

		D3D_DRIVER_TYPE driverTypes[] =
		{
			D3D_DRIVER_TYPE_HARDWARE,
			D3D_DRIVER_TYPE_WARP,
			D3D_DRIVER_TYPE_REFERENCE,
		};

		D3D_FEATURE_LEVEL featureLevels[] =
		{
			D3D_FEATURE_LEVEL_11_1,
			D3D_FEATURE_LEVEL_11_0,
			D3D_FEATURE_LEVEL_10_1,
			D3D_FEATURE_LEVEL_10_0,
		};

		for (UINT driverTypeIndex = 0; driverTypeIndex < ARRAYSIZE(driverTypes); driverTypeIndex++)
		{
			m_driverType = driverTypes[driverTypeIndex];
			hr = D3D11CreateDevice(nullptr, m_driverType, nullptr, createDeviceFlags, featureLevels, ARRAYSIZE(featureLevels),
				D3D11_SDK_VERSION, &m_pd3dDevice, &m_featureLevel, &m_pImmediateContext);

			if (hr == E_INVALIDARG)
			{
				hr = D3D11CreateDevice(nullptr, m_driverType, nullptr, createDeviceFlags, &featureLevels[1], ARRAYSIZE(featureLevels) - 1,
					D3D11_SDK_VERSION, &m_pd3dDevice, &m_featureLevel, &m_pImmediateContext);
			}

			if (SUCCEEDED(hr))
			{
				break;
			}
		}

		if (FAILED(hr))
		{
			return hr;
		}

		IDXGIFactory1* dxgiFactory = nullptr;
		{
			IDXGIDevice* dxgiDevice = nullptr;
			hr = m_pd3dDevice->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void**>(&dxgiDevice));
			if (SUCCEEDED(hr))
			{
				IDXGIAdapter* adapter = nullptr;
				hr = dxgiDevice->GetAdapter(&adapter);
				if (SUCCEEDED(hr))
				{
					hr = adapter->GetParent(__uuidof(IDXGIFactory1), reinterpret_cast<void**>(&dxgiFactory));
					adapter->Release();
				}
				dxgiDevice->Release();
			}
		}

		if (FAILED(hr))
		{
			return hr;
		}

		IDXGIFactory2* dxgiFactory2 = nullptr;
		hr = dxgiFactory->QueryInterface(__uuidof(IDXGIFactory2), reinterpret_cast<void**>(&dxgiFactory2));
		if (dxgiFactory2)
		{
			hr = m_pd3dDevice->QueryInterface(__uuidof(ID3D11Device1), reinterpret_cast<void**>(&m_pd3dDevice1));
			if (SUCCEEDED(hr))
			{
				(void)m_pImmediateContext->QueryInterface(__uuidof(ID3D11DeviceContext1), reinterpret_cast<void**>(&m_pImmediateContext1));
			}

			DXGI_SWAP_CHAIN_DESC1 sd = {};
			sd.Width = m_Width;
			sd.Height = m_Height;
			sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
			sd.SampleDesc.Count = 1;
			sd.SampleDesc.Quality = 0;
			sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
			sd.BufferCount = 1;

			hr = dxgiFactory2->CreateSwapChainForHwnd(m_pd3dDevice, hWnd, &sd, nullptr, nullptr, &m_pSwapChain1);
			if (SUCCEEDED(hr))
			{
				hr = m_pSwapChain1->QueryInterface(__uuidof(IDXGISwapChain), reinterpret_cast<void**>(&m_pSwapChain));
			}

			dxgiFactory2->Release();
		}
		else
		{
			DXGI_SWAP_CHAIN_DESC sd = {};
			sd.BufferCount = 1;
			sd.BufferDesc.Width = m_Width;
			sd.BufferDesc.Height = m_Height;
			sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
			sd.BufferDesc.RefreshRate.Numerator = 60;
			sd.BufferDesc.RefreshRate.Denominator = 1;
			sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
			sd.OutputWindow = hWnd;
			sd.SampleDesc.Count = 1;
			sd.SampleDesc.Quality = 0;
			sd.Windowed = TRUE;

			hr = dxgiFactory->CreateSwapChain(m_pd3dDevice, &sd, &m_pSwapChain);
		}

		dxgiFactory->MakeWindowAssociation(hWnd, DXGI_MWA_NO_ALT_ENTER);
		dxgiFactory->Release();

		if (FAILED(hr))
		{
			return hr;
		}

		hr = CreateBackBuffer();
		if (FAILED(hr))
		{
			return hr;
		}

		hr = CreateShadowResources();
		if (FAILED(hr))
		{
			return hr;
		}

		hr = CreateRasterizerStates();
		if (FAILED(hr))
		{
			return hr;
		}
		SetFillMode(false);
		SetDepthMode();
		hr = CreateOutlineStates();
		if (FAILED(hr))
		{
			return hr;
		}
		SetupViewports();

		hr = CreateShaders(shaderPath);
		if (FAILED(hr))
		{
			return hr;
		}

		SetCamera(Riemann::CameraDesc());
		SetLight(Riemann::DirectionalLightDesc());
		m_ImguiReady = m_ImguiRenderer.Init(m_pd3dDevice, m_pImmediateContext);
		return S_OK;
	}

	void Render() override
	{
		if (!m_pImmediateContext || !m_pSwapChain)
		{
			return;
		}
		if (!EnsureBackBufferSize())
		{
			return;
		}

		RenderShadowPass();
		RenderMainPass();
		RenderImgui();
		m_pSwapChain->Present(0, 0);
	}

	void SetCamera(const Riemann::CameraDesc& camera) override
	{
		m_Camera = camera;
		const float nearPlane = camera.NearPlane > 0.05f ? camera.NearPlane : 0.05f;
		const float farPlane = camera.FarPlane > nearPlane + 1.0f ? camera.FarPlane : nearPlane + 1.0f;
		m_Camera.NearPlane = nearPlane;
		m_Camera.FarPlane = farPlane;
		m_Eye = camera.Eye;
		m_View = Transform3::BuildViewMatrix_LHCoordinateSystem(camera.Eye, camera.At, camera.Up);
		const float aspect = m_Height != 0 ? m_Width / static_cast<float>(m_Height) : 1.0f;
		m_Projection = Transform3::BuildPerspectiveMatrix_LHCoordinateSystem(camera.FovY, aspect, nearPlane, farPlane);
	}

	void SetLight(const Riemann::DirectionalLightDesc& light) override
	{
		m_Light = light;
		m_Light.Direction = SafeUnit(light.Direction, Vector3(-0.4f, -1.0f, 0.35f));

		Vector3 up = Vector3::UnitY();
		if (fabsf(m_Light.Direction.Dot(up)) > 0.95f)
		{
			up = Vector3::UnitZ();
		}

		const Vector3 lightEye = m_Light.ShadowCenter - m_Light.Direction * m_Light.ShadowDistance;
		m_LightView = Transform3::BuildViewMatrix_LHCoordinateSystem(lightEye, m_Light.ShadowCenter, up);
		m_LightProjection = Transform3::BuildOrthogonalMatrix_LHCoordinateSystem(
			m_Light.ShadowSize,
			m_Light.ShadowSize,
			0.1f,
			m_Light.ShadowDistance * 2.0f);
	}

	bool AddMesh(const Riemann::RenderMeshDesc& meshDesc) override
	{
		if (meshDesc.Id.empty() || meshDesc.Vertices.empty() != meshDesc.Indices.empty())
		{
			return false;
		}

		if (meshDesc.Vertices.size() > (size_t)std::numeric_limits<int>::max()
			|| meshDesc.Indices.size() > (size_t)std::numeric_limits<int>::max()
			|| meshDesc.Vertices.size() > (size_t)std::numeric_limits<UINT>::max() / sizeof(Vertex1)
			|| meshDesc.Indices.size() > (size_t)std::numeric_limits<UINT>::max() / sizeof(uint32_t))
		{
			return false;
		}
		for (uint32_t index : meshDesc.Indices)
		{
			if (index >= meshDesc.Vertices.size()) return false;
		}
		if ((meshDesc.Topology == Riemann::RenderPrimitiveTopology::Triangles
				&& meshDesc.Indices.size() % 3 != 0)
			|| (meshDesc.Topology == Riemann::RenderPrimitiveTopology::Lines
				&& meshDesc.Indices.size() % 2 != 0))
		{
			return false;
		}

		DX11StaticMesh mesh;
		mesh.Id = meshDesc.Id;
		mesh.WorldTransform = meshDesc.WorldTransform;
		mesh.Color = meshDesc.Color;
		mesh.VertexCount = static_cast<int>(meshDesc.Vertices.size());
		mesh.IndexCount = static_cast<int>(meshDesc.Indices.size());
		mesh.VertexCapacity = mesh.VertexCount;
		mesh.IndexCapacity = mesh.IndexCount;
		mesh.IndexFormat = DXGI_FORMAT_R32_UINT;
		mesh.Topology = meshDesc.Topology == Riemann::RenderPrimitiveTopology::Lines
			? D3D11_PRIMITIVE_TOPOLOGY_LINELIST
			: D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		mesh.FillMode = meshDesc.FillMode;
		mesh.BlendMode = meshDesc.BlendMode;
		mesh.CastShadow = meshDesc.CastShadow && meshDesc.Topology == Riemann::RenderPrimitiveTopology::Triangles;
		mesh.Unlit = meshDesc.Unlit;

		HRESULT hr = S_OK;

		D3D11_BUFFER_DESC bd = {};
		D3D11_SUBRESOURCE_DATA initData = {};
		if (!meshDesc.Vertices.empty())
		{
			bd.Usage = D3D11_USAGE_DEFAULT;
			bd.ByteWidth = static_cast<UINT>(sizeof(Vertex1) * meshDesc.Vertices.size());
			bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
			initData.pSysMem = meshDesc.Vertices.data();
			hr = m_pd3dDevice->CreateBuffer(&bd, &initData, &mesh.pVertexBuffer);
			if (FAILED(hr)) return false;

			bd = {};
			bd.Usage = D3D11_USAGE_DEFAULT;
			bd.ByteWidth = static_cast<UINT>(sizeof(uint32_t) * meshDesc.Indices.size());
			bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
			initData = {};
			initData.pSysMem = meshDesc.Indices.data();
			hr = m_pd3dDevice->CreateBuffer(&bd, &initData, &mesh.pIndexBuffer);
			if (FAILED(hr)) return false;
		}

		bd = {};
		bd.Usage = D3D11_USAGE_DEFAULT;
		bd.ByteWidth = sizeof(ConstantBuffer);
		bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		hr = m_pd3dDevice->CreateBuffer(&bd, nullptr, &mesh.pConstantBuffer);
		if (FAILED(hr))
		{
			return false;
		}

		for (DX11StaticMesh& existing : m_AllMesh)
		{
			if (existing.Id == mesh.Id)
			{
				existing = std::move(mesh);
				return true;
			}
		}
		m_AllMesh.push_back(std::move(mesh));
		return true;
	}

	bool UpdateTransform(const Riemann::RenderTransformUpdate& transform) override
	{
		for (DX11StaticMesh& mesh : m_AllMesh)
		{
			if (mesh.Id == transform.Id)
			{
				mesh.WorldTransform = transform.WorldTransform;
				return true;
			}
		}
		return false;
	}

	bool UpdateVertices(const char* id, const Vertex1* vertices, int vertexCount) override
	{
		if (id == nullptr || vertices == nullptr || vertexCount <= 0
			|| (size_t)vertexCount > (size_t)std::numeric_limits<UINT>::max() / sizeof(Vertex1))
		{
			return false;
		}

		for (DX11StaticMesh& mesh : m_AllMesh)
		{
			if (mesh.Id == id)
			{
				if (vertexCount != mesh.VertexCount || mesh.pVertexBuffer == nullptr) return false;
				D3D11_BOX range = {};
				range.right = (UINT)(sizeof(Vertex1) * (size_t)vertexCount);
				range.bottom = 1;
				range.back = 1;
				m_pImmediateContext->UpdateSubresource(mesh.pVertexBuffer, 0, &range, vertices, 0, 0);
				return true;
			}
		}
		return false;
	}

	bool UpdateMeshGeometry(const char* id, const Vertex1* vertices, int vertexCount,
		const uint32_t* indices, int indexCount) override
	{
		if (id == nullptr || vertexCount < 0 || indexCount < 0
			|| ((vertexCount == 0 || indexCount == 0) && (vertexCount != 0 || indexCount != 0))
			|| (vertexCount > 0 && (vertices == nullptr || indices == nullptr))
			|| (size_t)vertexCount > (size_t)std::numeric_limits<UINT>::max() / sizeof(Vertex1)
			|| (size_t)indexCount > (size_t)std::numeric_limits<UINT>::max() / sizeof(uint32_t)
			|| m_pd3dDevice == nullptr || m_pImmediateContext == nullptr)
		{
			return false;
		}

		DX11StaticMesh* target = nullptr;
		for (DX11StaticMesh& mesh : m_AllMesh)
		{
			if (mesh.Id == id) { target = &mesh; break; }
		}
		if (target == nullptr) return false;
		if (vertexCount == 0)
		{
			target->VertexCount = 0;
			target->IndexCount = 0;
			return true;
		}
		if ((target->Topology == D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST && indexCount % 3 != 0)
			|| (target->Topology == D3D11_PRIMITIVE_TOPOLOGY_LINELIST && indexCount % 2 != 0))
		{
			return false;
		}
		for (int index = 0; index < indexCount; ++index)
		{
			if (indices[index] >= (uint32_t)vertexCount) return false;
		}

		auto GrownCapacity = [](int Current, int Required) -> int
		{
			if (Current <= 0) return Required;
			const int64_t Grown = (int64_t)Current + std::max<int64_t>(1, (int64_t)Current / 2);
			return (int)std::min<int64_t>(std::numeric_limits<int>::max(),
				std::max<int64_t>(Grown, Required));
		};
		const int vertexCapacity = vertexCount > target->VertexCapacity
			? GrownCapacity(target->VertexCapacity, vertexCount) : target->VertexCapacity;
		const int indexCapacity = indexCount > target->IndexCapacity
			? GrownCapacity(target->IndexCapacity, indexCount) : target->IndexCapacity;
		if ((size_t)vertexCapacity > (size_t)std::numeric_limits<UINT>::max() / sizeof(Vertex1)
			|| (size_t)indexCapacity > (size_t)std::numeric_limits<UINT>::max() / sizeof(uint32_t))
		{
			return false;
		}

		ID3D11Buffer* newVertexBuffer = nullptr;
		ID3D11Buffer* newIndexBuffer = nullptr;
		if (vertexCapacity != target->VertexCapacity)
		{
			D3D11_BUFFER_DESC desc = {};
			desc.Usage = D3D11_USAGE_DEFAULT;
			desc.ByteWidth = (UINT)(sizeof(Vertex1) * (size_t)vertexCapacity);
			desc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
			if (FAILED(m_pd3dDevice->CreateBuffer(&desc, nullptr, &newVertexBuffer))) return false;
		}
		if (indexCapacity != target->IndexCapacity)
		{
			D3D11_BUFFER_DESC desc = {};
			desc.Usage = D3D11_USAGE_DEFAULT;
			desc.ByteWidth = (UINT)(sizeof(uint32_t) * (size_t)indexCapacity);
			desc.BindFlags = D3D11_BIND_INDEX_BUFFER;
			if (FAILED(m_pd3dDevice->CreateBuffer(&desc, nullptr, &newIndexBuffer)))
			{
				SafeRelease(newVertexBuffer);
				return false;
			}
		}

		ID3D11Buffer* vertexBuffer = newVertexBuffer ? newVertexBuffer : target->pVertexBuffer;
		ID3D11Buffer* indexBuffer = newIndexBuffer ? newIndexBuffer : target->pIndexBuffer;
		if (vertexBuffer == nullptr || indexBuffer == nullptr)
		{
			SafeRelease(newVertexBuffer); SafeRelease(newIndexBuffer); return false;
		}
		D3D11_BOX vertexRange = {};
		vertexRange.right = (UINT)(sizeof(Vertex1) * (size_t)vertexCount);
		vertexRange.bottom = 1; vertexRange.back = 1;
		m_pImmediateContext->UpdateSubresource(vertexBuffer, 0, &vertexRange, vertices, 0, 0);
		D3D11_BOX indexRange = {};
		indexRange.right = (UINT)(sizeof(uint32_t) * (size_t)indexCount);
		indexRange.bottom = 1; indexRange.back = 1;
		m_pImmediateContext->UpdateSubresource(indexBuffer, 0, &indexRange, indices, 0, 0);

		if (newVertexBuffer)
		{
			SafeRelease(target->pVertexBuffer); target->pVertexBuffer = newVertexBuffer;
			target->VertexCapacity = vertexCapacity;
		}
		if (newIndexBuffer)
		{
			SafeRelease(target->pIndexBuffer); target->pIndexBuffer = newIndexBuffer;
			target->IndexCapacity = indexCapacity;
		}
		target->VertexCount = vertexCount;
		target->IndexCount = indexCount;
		return true;
	}

	bool DeleteMesh(const char* id) override
	{
		if (id == nullptr)
		{
			return false;
		}

		for (size_t i = 0; i < m_AllMesh.size(); ++i)
		{
			if (m_AllMesh[i].Id == id)
			{
				m_AllMesh.erase(m_AllMesh.begin() + i);
				return true;
			}
		}
		return false;
	}

	bool SetMeshOutline(const char* id, bool enabled, const Vector4& color, float thickness) override
	{
		if (id == nullptr)
		{
			return false;
		}

		for (DX11StaticMesh& mesh : m_AllMesh)
		{
			if (mesh.Id == id)
			{
				mesh.OutlineEnabled = enabled;
				mesh.OutlineColor = color;
				mesh.OutlineThickness = thickness > 0.0f ? thickness : 0.0f;
				return true;
			}
		}
		return false;
	}

	bool Reset() override
	{
		m_AllMesh.clear();
		return true;
	}

	void SetFillMode(bool wireframe) override
	{
		m_DefaultWireframe = wireframe;
		if (m_pImmediateContext)
		{
			m_pImmediateContext->RSSetState(GetRasterizerState(Riemann::RenderFillMode::RendererDefault));
		}
	}

	void SetDepthMode() override
	{
		SafeRelease(m_pDepthStencilState);
		SafeRelease(m_pOutlineMaskDepthStencilState);
		SafeRelease(m_pOutlineDepthStencilState);
		SafeRelease(m_pTransparentDepthStencilState);
		SafeRelease(m_pAdditiveBlendState);
		SafeRelease(m_pTransparentBlendState);
		SafeRelease(m_pColorWriteOffBlendState);

		D3D11_DEPTH_STENCIL_DESC desc = {};
		desc.DepthEnable = TRUE;
		desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
		desc.DepthFunc = D3D11_COMPARISON_LESS;
		desc.StencilEnable = FALSE;

		if (SUCCEEDED(m_pd3dDevice->CreateDepthStencilState(&desc, &m_pDepthStencilState)))
		{
			m_pImmediateContext->OMSetDepthStencilState(m_pDepthStencilState, 0);
		}

		D3D11_DEPTH_STENCIL_DESC transparentDepthDesc = desc;
		transparentDepthDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
		(void)m_pd3dDevice->CreateDepthStencilState(&transparentDepthDesc, &m_pTransparentDepthStencilState);

		D3D11_BLEND_DESC transparentBlendDesc = {};
		transparentBlendDesc.RenderTarget[0].BlendEnable = TRUE;
		transparentBlendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
		transparentBlendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
		transparentBlendDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
		transparentBlendDesc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
		transparentBlendDesc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
		transparentBlendDesc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
		transparentBlendDesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
		(void)m_pd3dDevice->CreateBlendState(&transparentBlendDesc, &m_pTransparentBlendState);

		D3D11_BLEND_DESC additiveBlendDesc = transparentBlendDesc;
		additiveBlendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_ONE;
		additiveBlendDesc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ONE;
		(void)m_pd3dDevice->CreateBlendState(&additiveBlendDesc, &m_pAdditiveBlendState);

		D3D11_DEPTH_STENCIL_DESC maskDesc = {};
		maskDesc.DepthEnable = FALSE;
		maskDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
		maskDesc.DepthFunc = D3D11_COMPARISON_ALWAYS;
		maskDesc.StencilEnable = TRUE;
		maskDesc.StencilReadMask = 0xff;
		maskDesc.StencilWriteMask = 0xff;
		maskDesc.FrontFace.StencilFailOp = D3D11_STENCIL_OP_KEEP;
		maskDesc.FrontFace.StencilDepthFailOp = D3D11_STENCIL_OP_KEEP;
		maskDesc.FrontFace.StencilPassOp = D3D11_STENCIL_OP_REPLACE;
		maskDesc.FrontFace.StencilFunc = D3D11_COMPARISON_ALWAYS;
		maskDesc.BackFace = maskDesc.FrontFace;
		(void)m_pd3dDevice->CreateDepthStencilState(&maskDesc, &m_pOutlineMaskDepthStencilState);

		D3D11_DEPTH_STENCIL_DESC outlineDesc = {};
		outlineDesc.DepthEnable = FALSE;
		outlineDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
		outlineDesc.DepthFunc = D3D11_COMPARISON_ALWAYS;
		outlineDesc.StencilEnable = TRUE;
		outlineDesc.StencilReadMask = 0xff;
		outlineDesc.StencilWriteMask = 0x00;
		outlineDesc.FrontFace.StencilFailOp = D3D11_STENCIL_OP_KEEP;
		outlineDesc.FrontFace.StencilDepthFailOp = D3D11_STENCIL_OP_KEEP;
		outlineDesc.FrontFace.StencilPassOp = D3D11_STENCIL_OP_KEEP;
		outlineDesc.FrontFace.StencilFunc = D3D11_COMPARISON_NOT_EQUAL;
		outlineDesc.BackFace = outlineDesc.FrontFace;
		(void)m_pd3dDevice->CreateDepthStencilState(&outlineDesc, &m_pOutlineDepthStencilState);

		D3D11_BLEND_DESC blendDesc = {};
		blendDesc.RenderTarget[0].BlendEnable = FALSE;
		blendDesc.RenderTarget[0].RenderTargetWriteMask = 0;
		(void)m_pd3dDevice->CreateBlendState(&blendDesc, &m_pColorWriteOffBlendState);
	}

	void SetImguiDrawCallback(Riemann::ImguiDrawCallback callback, void* userData) override
	{
		std::lock_guard<std::mutex> lock(m_ImguiMutex);
		m_ImguiCallback = callback;
		m_ImguiUserData = userData;
	}

	void UpdateImguiInput(const Riemann::ImguiInputState& input) override
	{
		std::lock_guard<std::mutex> lock(m_ImguiMutex);
		m_ImguiInput.MouseX = input.MouseX;
		m_ImguiInput.MouseY = input.MouseY;
		m_ImguiInput.MouseButtons = input.MouseButtons;
		m_ImguiScroll += input.MouseWheel;
	}

private:
	HRESULT CreateBackBuffer()
	{
		ID3D11Texture2D* pBackBuffer = nullptr;
		HRESULT hr = m_pSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&pBackBuffer));
		if (FAILED(hr))
		{
			return hr;
		}

		hr = m_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &m_pRenderTargetView);
		pBackBuffer->Release();
		if (FAILED(hr))
		{
			return hr;
		}

		D3D11_TEXTURE2D_DESC depthTextureDesc = {};
		depthTextureDesc.Width = m_Width;
		depthTextureDesc.Height = m_Height;
		depthTextureDesc.MipLevels = 1;
		depthTextureDesc.ArraySize = 1;
		depthTextureDesc.SampleDesc.Count = 1;
		depthTextureDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
		depthTextureDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;

		ID3D11Texture2D* depthStencilTexture = nullptr;
		hr = m_pd3dDevice->CreateTexture2D(&depthTextureDesc, nullptr, &depthStencilTexture);
		if (FAILED(hr))
		{
			return hr;
		}

		D3D11_DEPTH_STENCIL_VIEW_DESC dsvDesc = {};
		dsvDesc.Format = depthTextureDesc.Format;
		dsvDesc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;

		hr = m_pd3dDevice->CreateDepthStencilView(depthStencilTexture, &dsvDesc, &m_pDepthBuffer);
		depthStencilTexture->Release();
		return hr;
	}

	HRESULT CreateShadowResources()
	{
		D3D11_TEXTURE2D_DESC texDesc = {};
		texDesc.Width = m_ShadowMapSize;
		texDesc.Height = m_ShadowMapSize;
		texDesc.MipLevels = 1;
		texDesc.ArraySize = 1;
		texDesc.Format = DXGI_FORMAT_R32_TYPELESS;
		texDesc.SampleDesc.Count = 1;
		texDesc.Usage = D3D11_USAGE_DEFAULT;
		texDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL | D3D11_BIND_SHADER_RESOURCE;

		HRESULT hr = m_pd3dDevice->CreateTexture2D(&texDesc, nullptr, &m_pShadowTexture);
		if (FAILED(hr))
		{
			return hr;
		}

		D3D11_DEPTH_STENCIL_VIEW_DESC dsvDesc = {};
		dsvDesc.Format = DXGI_FORMAT_D32_FLOAT;
		dsvDesc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
		hr = m_pd3dDevice->CreateDepthStencilView(m_pShadowTexture, &dsvDesc, &m_pShadowDepthView);
		if (FAILED(hr))
		{
			return hr;
		}

		D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
		srvDesc.Format = DXGI_FORMAT_R32_FLOAT;
		srvDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
		srvDesc.Texture2D.MipLevels = 1;
		hr = m_pd3dDevice->CreateShaderResourceView(m_pShadowTexture, &srvDesc, &m_pShadowSRV);
		if (FAILED(hr))
		{
			return hr;
		}

		D3D11_SAMPLER_DESC samplerDesc = {};
		samplerDesc.Filter = D3D11_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT;
		samplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_BORDER;
		samplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_BORDER;
		samplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_BORDER;
		samplerDesc.BorderColor[0] = 1.0f;
		samplerDesc.BorderColor[1] = 1.0f;
		samplerDesc.BorderColor[2] = 1.0f;
		samplerDesc.BorderColor[3] = 1.0f;
		samplerDesc.ComparisonFunc = D3D11_COMPARISON_LESS_EQUAL;
		samplerDesc.MinLOD = 0.0f;
		samplerDesc.MaxLOD = D3D11_FLOAT32_MAX;
		return m_pd3dDevice->CreateSamplerState(&samplerDesc, &m_pShadowSampler);
	}

	HRESULT CreateRasterizerStates()
	{
		SafeRelease(m_pSolidRasterizerState);
		SafeRelease(m_pWireframeRasterizerState);

		D3D11_RASTERIZER_DESC desc = {};
		desc.CullMode = D3D11_CULL_NONE;
		desc.DepthClipEnable = TRUE;
		desc.MultisampleEnable = FALSE;
		desc.AntialiasedLineEnable = FALSE;

		desc.FillMode = D3D11_FILL_SOLID;
		HRESULT hr = m_pd3dDevice->CreateRasterizerState(&desc, &m_pSolidRasterizerState);
		if (FAILED(hr))
		{
			return hr;
		}

		desc.FillMode = D3D11_FILL_WIREFRAME;
		return m_pd3dDevice->CreateRasterizerState(&desc, &m_pWireframeRasterizerState);
	}

	ID3D11RasterizerState* GetRasterizerState(Riemann::RenderFillMode fillMode) const
	{
		return IsWireframeFillMode(fillMode) ? m_pWireframeRasterizerState : m_pSolidRasterizerState;
	}

	bool IsWireframeFillMode(Riemann::RenderFillMode fillMode) const
	{
		return fillMode == Riemann::RenderFillMode::Wireframe ||
			(fillMode == Riemann::RenderFillMode::RendererDefault && m_DefaultWireframe);
	}

	bool ShouldRenderUnlit(const DX11StaticMesh& mesh, bool shadowPass, bool outlinePass) const
	{
		if (shadowPass || outlinePass)
		{
			return false;
		}
		return mesh.Unlit || mesh.Topology == D3D11_PRIMITIVE_TOPOLOGY_LINELIST ||
			(mesh.Topology == D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST && IsWireframeFillMode(mesh.FillMode));
	}

	HRESULT CreateOutlineStates()
	{
		SafeRelease(m_pOutlineRasterizerState);

		D3D11_RASTERIZER_DESC rasterDesc = {};
		rasterDesc.FillMode = D3D11_FILL_SOLID;
		rasterDesc.CullMode = D3D11_CULL_FRONT;
		rasterDesc.DepthClipEnable = TRUE;
		rasterDesc.MultisampleEnable = FALSE;
		rasterDesc.AntialiasedLineEnable = FALSE;

		return m_pd3dDevice->CreateRasterizerState(&rasterDesc, &m_pOutlineRasterizerState);
	}

	void SetupViewports()
	{
		m_BackBufferViewport = {};
		m_BackBufferViewport.Width = static_cast<FLOAT>(m_Width);
		m_BackBufferViewport.Height = static_cast<FLOAT>(m_Height);
		m_BackBufferViewport.MinDepth = 0.0f;
		m_BackBufferViewport.MaxDepth = 1.0f;
		m_BackBufferViewport.TopLeftX = 0.0f;
		m_BackBufferViewport.TopLeftY = 0.0f;

		m_ShadowViewport = {};
		m_ShadowViewport.Width = static_cast<FLOAT>(m_ShadowMapSize);
		m_ShadowViewport.Height = static_cast<FLOAT>(m_ShadowMapSize);
		m_ShadowViewport.MinDepth = 0.0f;
		m_ShadowViewport.MaxDepth = 1.0f;
		m_ShadowViewport.TopLeftX = 0.0f;
		m_ShadowViewport.TopLeftY = 0.0f;
	}

	HRESULT CreateShaders(const char* shaderPath)
	{
		std::wstring shaderFile = ToWidePath(shaderPath);
		if (!shaderFile.empty())
		{
			const wchar_t last = shaderFile[shaderFile.size() - 1];
			if (last != L'/' && last != L'\\')
			{
				shaderFile.push_back(L'/');
			}
		}
		shaderFile += L"dx11_shader.hlsl";

		ID3DBlob* pVSBlob = nullptr;
		HRESULT hr = CompileShaderFromFile(shaderFile.c_str(), "MainVS", "vs_4_0", &pVSBlob);
		if (FAILED(hr))
		{
			MessageBoxA(nullptr, "Compile VS Error", "Renderer", MB_OK);
			return hr;
		}

		hr = m_pd3dDevice->CreateVertexShader(pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), nullptr, &m_pVertexShader);
		if (FAILED(hr))
		{
			pVSBlob->Release();
			return hr;
		}

		D3D11_INPUT_ELEMENT_DESC layout[] =
		{
			{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		};

		hr = m_pd3dDevice->CreateInputLayout(layout, ARRAYSIZE(layout), pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), &m_pVertexLayout);
		pVSBlob->Release();
		if (FAILED(hr))
		{
			return hr;
		}

		ID3DBlob* pShadowVSBlob = nullptr;
		hr = CompileShaderFromFile(shaderFile.c_str(), "ShadowVS", "vs_4_0", &pShadowVSBlob);
		if (FAILED(hr))
		{
			MessageBoxA(nullptr, "Compile Shadow VS Error", "Renderer", MB_OK);
			return hr;
		}

		hr = m_pd3dDevice->CreateVertexShader(pShadowVSBlob->GetBufferPointer(), pShadowVSBlob->GetBufferSize(), nullptr, &m_pShadowVertexShader);
		pShadowVSBlob->Release();
		if (FAILED(hr))
		{
			return hr;
		}

		ID3DBlob* pOutlineVSBlob = nullptr;
		hr = CompileShaderFromFile(shaderFile.c_str(), "OutlineVS", "vs_4_0", &pOutlineVSBlob);
		if (FAILED(hr))
		{
			MessageBoxA(nullptr, "Compile Outline VS Error", "Renderer", MB_OK);
			return hr;
		}

		hr = m_pd3dDevice->CreateVertexShader(pOutlineVSBlob->GetBufferPointer(), pOutlineVSBlob->GetBufferSize(), nullptr, &m_pOutlineVertexShader);
		pOutlineVSBlob->Release();
		if (FAILED(hr))
		{
			return hr;
		}

		ID3DBlob* pPSBlob = nullptr;
		hr = CompileShaderFromFile(shaderFile.c_str(), "MainPS", "ps_4_0", &pPSBlob);
		if (FAILED(hr))
		{
			MessageBoxA(nullptr, "Compile PS Error", "Renderer", MB_OK);
			return hr;
		}

		hr = m_pd3dDevice->CreatePixelShader(pPSBlob->GetBufferPointer(), pPSBlob->GetBufferSize(), nullptr, &m_pPixelShader);
		pPSBlob->Release();
		if (FAILED(hr))
		{
			return hr;
		}

		ID3DBlob* pOutlinePSBlob = nullptr;
		hr = CompileShaderFromFile(shaderFile.c_str(), "OutlinePS", "ps_4_0", &pOutlinePSBlob);
		if (FAILED(hr))
		{
			MessageBoxA(nullptr, "Compile Outline PS Error", "Renderer", MB_OK);
			return hr;
		}

		hr = m_pd3dDevice->CreatePixelShader(pOutlinePSBlob->GetBufferPointer(), pOutlinePSBlob->GetBufferSize(), nullptr, &m_pOutlinePixelShader);
		pOutlinePSBlob->Release();
		return hr;
	}

	static HRESULT CompileShaderFromFile(const WCHAR* fileName, LPCSTR entryPoint, LPCSTR shaderModel, ID3DBlob** ppBlobOut)
	{
		DWORD shaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
#ifdef _DEBUG
		shaderFlags |= D3DCOMPILE_DEBUG;
		shaderFlags |= D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

		ID3DBlob* pErrorBlob = nullptr;
		HRESULT hr = D3DCompileFromFile(fileName, nullptr, nullptr, entryPoint, shaderModel, shaderFlags, 0, ppBlobOut, &pErrorBlob);
		if (FAILED(hr))
		{
			if (pErrorBlob)
			{
				OutputDebugStringA(reinterpret_cast<const char*>(pErrorBlob->GetBufferPointer()));
				pErrorBlob->Release();
			}
			return hr;
		}

		if (pErrorBlob)
		{
			pErrorBlob->Release();
		}
		return S_OK;
	}

	bool EnsureBackBufferSize()
	{
		if (m_hWnd == nullptr || m_pSwapChain == nullptr)
		{
			return true;
		}

		RECT rc = {};
		if (!GetClientRect(m_hWnd, &rc))
		{
			return true;
		}

		const UINT width = rc.right > rc.left ? static_cast<UINT>(rc.right - rc.left) : 0;
		const UINT height = rc.bottom > rc.top ? static_cast<UINT>(rc.bottom - rc.top) : 0;
		if (width == 0 || height == 0)
		{
			return false;
		}

		if (width == m_Width && height == m_Height && m_pRenderTargetView != nullptr && m_pDepthBuffer != nullptr)
		{
			return true;
		}

		m_pImmediateContext->OMSetRenderTargets(0, nullptr, nullptr);
		ID3D11ShaderResourceView* nullSRV[] = { nullptr };
		m_pImmediateContext->PSSetShaderResources(0, 1, nullSRV);
		SafeRelease(m_pRenderTargetView);
		SafeRelease(m_pDepthBuffer);

		HRESULT hr = m_pSwapChain->ResizeBuffers(0, width, height, DXGI_FORMAT_UNKNOWN, 0);
		if (FAILED(hr))
		{
			return false;
		}

		m_Width = width;
		m_Height = height;
		hr = CreateBackBuffer();
		if (FAILED(hr))
		{
			return false;
		}

		SetupViewports();
		SetCamera(m_Camera);
		return true;
	}

	void RenderShadowPass()
	{
		ID3D11ShaderResourceView* nullSRV[] = { nullptr };
		m_pImmediateContext->PSSetShaderResources(0, 1, nullSRV);

		const float blendFactor[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
		m_pImmediateContext->OMSetBlendState(nullptr, blendFactor, 0xffffffffu);
		m_pImmediateContext->OMSetDepthStencilState(m_pDepthStencilState, 0);
		m_pImmediateContext->RSSetState(GetRasterizerState(Riemann::RenderFillMode::RendererDefault));
		m_pImmediateContext->RSSetViewports(1, &m_ShadowViewport);
		m_pImmediateContext->OMSetRenderTargets(0, nullptr, m_pShadowDepthView);
		m_pImmediateContext->ClearDepthStencilView(m_pShadowDepthView, D3D11_CLEAR_DEPTH, 1.0f, 0);
		if (!m_Light.ShadowsEnabled)
		{
			return;
		}

		m_pImmediateContext->IASetInputLayout(m_pVertexLayout);
		m_pImmediateContext->VSSetShader(m_pShadowVertexShader, nullptr, 0);
		m_pImmediateContext->PSSetShader(nullptr, nullptr, 0);

		DrawScene(true);
	}

	void RenderMainPass()
	{
		const float clearColor[4] = {
			m_Camera.BackgroundColor.x,
			m_Camera.BackgroundColor.y,
			m_Camera.BackgroundColor.z,
			m_Camera.BackgroundColor.w
		};
		const float blendFactor[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
		m_pImmediateContext->OMSetBlendState(nullptr, blendFactor, 0xffffffffu);
		m_pImmediateContext->OMSetDepthStencilState(m_pDepthStencilState, 0);
		m_pImmediateContext->RSSetState(GetRasterizerState(Riemann::RenderFillMode::RendererDefault));
		m_pImmediateContext->RSSetViewports(1, &m_BackBufferViewport);
		m_pImmediateContext->OMSetRenderTargets(1, &m_pRenderTargetView, m_pDepthBuffer);
		m_pImmediateContext->ClearRenderTargetView(m_pRenderTargetView, clearColor);
		m_pImmediateContext->ClearDepthStencilView(m_pDepthBuffer, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

		m_pImmediateContext->IASetInputLayout(m_pVertexLayout);
		m_pImmediateContext->VSSetShader(m_pVertexShader, nullptr, 0);
		m_pImmediateContext->PSSetShader(m_pPixelShader, nullptr, 0);
		m_pImmediateContext->PSSetShaderResources(0, 1, &m_pShadowSRV);
		m_pImmediateContext->PSSetSamplers(0, 1, &m_pShadowSampler);

		DrawScene(false);
		RenderOutlinePass();

		ID3D11ShaderResourceView* nullSRV[] = { nullptr };
		m_pImmediateContext->PSSetShaderResources(0, 1, nullSRV);
	}

	void RenderOutlinePass()
	{
		if (m_pOutlineVertexShader == nullptr || m_pOutlinePixelShader == nullptr ||
			m_pOutlineMaskDepthStencilState == nullptr || m_pOutlineDepthStencilState == nullptr ||
			m_pColorWriteOffBlendState == nullptr)
		{
			return;
		}

		bool hasOutline = false;
		for (const DX11StaticMesh& mesh : m_AllMesh)
		{
			if (mesh.OutlineEnabled && mesh.Topology == D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST)
			{
				hasOutline = true;
				break;
			}
		}
		if (!hasOutline)
		{
			return;
		}

		const float blendFactor[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
		m_pImmediateContext->OMSetBlendState(m_pColorWriteOffBlendState, blendFactor, 0xffffffffu);
		m_pImmediateContext->OMSetDepthStencilState(m_pOutlineMaskDepthStencilState, 1);
		m_pImmediateContext->RSSetState(GetRasterizerState(Riemann::RenderFillMode::Solid));
		m_pImmediateContext->IASetInputLayout(m_pVertexLayout);
		m_pImmediateContext->VSSetShader(m_pVertexShader, nullptr, 0);
		m_pImmediateContext->PSSetShader(nullptr, nullptr, 0);

		for (DX11StaticMesh& mesh : m_AllMesh)
		{
			if (!mesh.OutlineEnabled || mesh.Topology != D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST)
			{
				continue;
			}

			DrawMesh(mesh, false, true);
		}

		m_pImmediateContext->OMSetBlendState(nullptr, blendFactor, 0xffffffffu);
		m_pImmediateContext->OMSetDepthStencilState(m_pOutlineDepthStencilState, 1);
		m_pImmediateContext->RSSetState(m_pOutlineRasterizerState);
		m_pImmediateContext->VSSetShader(m_pOutlineVertexShader, nullptr, 0);
		m_pImmediateContext->PSSetShader(m_pOutlinePixelShader, nullptr, 0);

		for (DX11StaticMesh& mesh : m_AllMesh)
		{
			if (!mesh.OutlineEnabled || mesh.Topology != D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST)
			{
				continue;
			}

			DrawMesh(mesh, false, true);
		}

		m_pImmediateContext->OMSetDepthStencilState(m_pDepthStencilState, 0);
		m_pImmediateContext->RSSetState(GetRasterizerState(Riemann::RenderFillMode::RendererDefault));
		m_pImmediateContext->VSSetShader(m_pVertexShader, nullptr, 0);
		m_pImmediateContext->PSSetShader(m_pPixelShader, nullptr, 0);
		m_pImmediateContext->OMSetBlendState(nullptr, blendFactor, 0xffffffffu);
	}

	void RenderImgui()
	{
		if (!m_ImguiReady)
		{
			return;
		}

		Riemann::ImguiDrawCallback callback = nullptr;
		void* userData = nullptr;
		Riemann::ImguiInputState input;
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			callback = m_ImguiCallback;
			userData = m_ImguiUserData;
			input = m_ImguiInput;
			input.MouseWheel = m_ImguiScroll;
			m_ImguiScroll = 0;
		}

		if (callback == nullptr)
		{
			return;
		}

		input.MouseY = static_cast<int>(m_Height) - 1 - input.MouseY;
		imguiBeginFrame(input.MouseX, input.MouseY, input.MouseButtons, input.MouseWheel);
		callback(static_cast<int>(m_Width), static_cast<int>(m_Height), userData);
		imguiEndFrame();
		m_ImguiRenderer.Draw(static_cast<int>(m_Width), static_cast<int>(m_Height));
	}

	void DrawScene(bool shadowPass)
	{
		if (!shadowPass)
		{
			for (DX11StaticMesh& mesh : m_AllMesh)
			{
				if (mesh.BlendMode == Riemann::RenderBlendMode::Alpha && mesh.Color.w >= 0.999f)
				{
					DrawMesh(mesh, false, false);
				}
			}

			std::vector<DX11StaticMesh*> transparentMeshes;
			for (DX11StaticMesh& mesh : m_AllMesh)
			{
				if (mesh.BlendMode == Riemann::RenderBlendMode::Alpha && mesh.Color.w < 0.999f)
				{
					transparentMeshes.push_back(&mesh);
				}
			}
			std::sort(transparentMeshes.begin(), transparentMeshes.end(), [this](const DX11StaticMesh* lhs, const DX11StaticMesh* rhs)
			{
				return (lhs->WorldTransform.pos - m_Eye).SquareLength() > (rhs->WorldTransform.pos - m_Eye).SquareLength();
			});

			const float blendFactor[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
			m_pImmediateContext->OMSetBlendState(m_pTransparentBlendState, blendFactor, 0xffffffffu);
			m_pImmediateContext->OMSetDepthStencilState(m_pTransparentDepthStencilState, 0);
			for (DX11StaticMesh* mesh : transparentMeshes)
			{
				DrawMesh(*mesh, false, false);
			}

			m_pImmediateContext->OMSetBlendState(m_pAdditiveBlendState, blendFactor, 0xffffffffu);
			for (DX11StaticMesh& mesh : m_AllMesh)
			{
				if (mesh.BlendMode == Riemann::RenderBlendMode::Additive)
				{
					DrawMesh(mesh, false, false);
				}
			}
			m_pImmediateContext->OMSetBlendState(nullptr, blendFactor, 0xffffffffu);
			m_pImmediateContext->OMSetDepthStencilState(m_pDepthStencilState, 0);
			return;
		}

		for (DX11StaticMesh& mesh : m_AllMesh)
		{
			if (shadowPass && !mesh.CastShadow)
			{
				continue;
			}

			DrawMesh(mesh, shadowPass, false);
		}
	}

	void DrawMesh(DX11StaticMesh& mesh, bool shadowPass, bool outlinePass)
	{
		if (mesh.IndexCount == 0 || mesh.pVertexBuffer == nullptr || mesh.pIndexBuffer == nullptr)
		{
			return;
		}
		ConstantBuffer cb = {};
		cb.World = GetTransformMatrix(mesh.WorldTransform);
		cb.View = m_View;
		cb.Projection = m_Projection;
		cb.LightView = m_LightView;
		cb.LightProjection = m_LightProjection;
		cb.EyePos = Vector4(m_Eye.x, m_Eye.y, m_Eye.z, 1.0f);
		cb.LightDir = Vector4(m_Light.Direction.x, m_Light.Direction.y, m_Light.Direction.z, m_Light.Ambient);
		cb.LightColor = Vector4(m_Light.Color.x, m_Light.Color.y, m_Light.Color.z, 1.0f);
		cb.MaterialColor = mesh.Color;
		cb.OutlineColor = mesh.OutlineColor;
		cb.RenderParams = Vector4(outlinePass ? mesh.OutlineThickness : 0.0f, ShouldRenderUnlit(mesh, shadowPass, outlinePass) ? 1.0f : 0.0f, 0.0f, 0.0f);

		m_pImmediateContext->UpdateSubresource(mesh.pConstantBuffer, 0, nullptr, &cb, 0, 0);
		m_pImmediateContext->VSSetConstantBuffers(0, 1, &mesh.pConstantBuffer);
		if (!shadowPass)
		{
			m_pImmediateContext->PSSetConstantBuffers(0, 1, &mesh.pConstantBuffer);
		}
		if (!outlinePass)
		{
			m_pImmediateContext->RSSetState(GetRasterizerState(mesh.FillMode));
		}

		UINT stride = sizeof(Vertex1);
		UINT offset = 0;
		m_pImmediateContext->IASetVertexBuffers(0, 1, &mesh.pVertexBuffer, &stride, &offset);
		m_pImmediateContext->IASetIndexBuffer(mesh.pIndexBuffer, mesh.IndexFormat, 0);
		m_pImmediateContext->IASetPrimitiveTopology(mesh.Topology);
		m_pImmediateContext->DrawIndexed(mesh.IndexCount, 0, 0);
	}

private:
	D3D_DRIVER_TYPE m_driverType = D3D_DRIVER_TYPE_NULL;
	D3D_FEATURE_LEVEL m_featureLevel = D3D_FEATURE_LEVEL_11_0;
	ID3D11Device* m_pd3dDevice = nullptr;
	ID3D11Device1* m_pd3dDevice1 = nullptr;
	ID3D11DeviceContext* m_pImmediateContext = nullptr;
	ID3D11DeviceContext1* m_pImmediateContext1 = nullptr;
	IDXGISwapChain* m_pSwapChain = nullptr;
	IDXGISwapChain1* m_pSwapChain1 = nullptr;
	HWND m_hWnd = nullptr;
	ID3D11RenderTargetView* m_pRenderTargetView = nullptr;
	ID3D11DepthStencilView* m_pDepthBuffer = nullptr;
	ID3D11VertexShader* m_pVertexShader = nullptr;
	ID3D11VertexShader* m_pShadowVertexShader = nullptr;
	ID3D11PixelShader* m_pPixelShader = nullptr;
	ID3D11VertexShader* m_pOutlineVertexShader = nullptr;
	ID3D11PixelShader* m_pOutlinePixelShader = nullptr;
	ID3D11InputLayout* m_pVertexLayout = nullptr;
	ID3D11RasterizerState* m_pSolidRasterizerState = nullptr;
	ID3D11RasterizerState* m_pWireframeRasterizerState = nullptr;
	ID3D11RasterizerState* m_pOutlineRasterizerState = nullptr;
	ID3D11DepthStencilState* m_pDepthStencilState = nullptr;
	ID3D11DepthStencilState* m_pTransparentDepthStencilState = nullptr;
	ID3D11DepthStencilState* m_pOutlineMaskDepthStencilState = nullptr;
	ID3D11DepthStencilState* m_pOutlineDepthStencilState = nullptr;
	ID3D11BlendState* m_pColorWriteOffBlendState = nullptr;
	ID3D11BlendState* m_pAdditiveBlendState = nullptr;
	ID3D11BlendState* m_pTransparentBlendState = nullptr;
	ID3D11Texture2D* m_pShadowTexture = nullptr;
	ID3D11DepthStencilView* m_pShadowDepthView = nullptr;
	ID3D11ShaderResourceView* m_pShadowSRV = nullptr;
	ID3D11SamplerState* m_pShadowSampler = nullptr;
	Riemann::ImguiRenderDX11 m_ImguiRenderer;

	UINT m_Width = 0;
	UINT m_Height = 0;
	UINT m_ShadowMapSize = 2048;
	D3D11_VIEWPORT m_BackBufferViewport = {};
	D3D11_VIEWPORT m_ShadowViewport = {};

	Riemann::CameraDesc m_Camera;
	Riemann::DirectionalLightDesc m_Light;
	Matrix4 m_View;
	Matrix4 m_Projection;
	Matrix4 m_LightView;
	Matrix4 m_LightProjection;
	Vector3 m_Eye;

	std::vector<DX11StaticMesh> m_AllMesh;
	std::mutex m_ImguiMutex;
	Riemann::ImguiInputState m_ImguiInput;
	Riemann::ImguiDrawCallback m_ImguiCallback = nullptr;
	void* m_ImguiUserData = nullptr;
	int m_ImguiScroll = 0;
	bool m_ImguiReady = false;
	bool m_DefaultWireframe = false;
};

namespace Riemann
{
	Renderer* Renderer::CreateDX11Renderer(void* hWnd, const char* shaderPath)
	{
		DX11Renderer* renderer = new DX11Renderer();
		if (renderer->InitDevice(reinterpret_cast<HWND>(hWnd), shaderPath) == S_OK)
		{
			return renderer;
		}

		delete renderer;
		return nullptr;
	}
}

#endif
