#include "DX11Renderer.h"

#if defined(_WIN32)

#include <assert.h>
#include <string>
#include <utility>
#include <vector>
#include <windows.h>

#include <d3d11_1.h>
#include <d3dcompiler.h>
#include <directxcolors.h>

#include "../Src/Maths/Matrix4.h"
#include "../Src/Maths/Quaternion.h"
#include "../Src/Maths/Transform.h"
#include "../Src/Maths/Vector4.h"

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
			IndexCount = rhs.IndexCount;
			IndexFormat = rhs.IndexFormat;
			Topology = rhs.Topology;
			CastShadow = rhs.CastShadow;

			rhs.pVertexBuffer = nullptr;
			rhs.pIndexBuffer = nullptr;
			rhs.pConstantBuffer = nullptr;
			rhs.IndexCount = 0;
		}

		std::string Id;
		ID3D11Buffer* pVertexBuffer = nullptr;
		ID3D11Buffer* pIndexBuffer = nullptr;
		ID3D11Buffer* pConstantBuffer = nullptr;
		Transform WorldTransform;
		Vector4 Color = Vector4(0.72f, 0.76f, 0.82f, 1.0f);
		int IndexCount = 0;
		DXGI_FORMAT IndexFormat = DXGI_FORMAT_R32_UINT;
		D3D11_PRIMITIVE_TOPOLOGY Topology = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		bool CastShadow = true;
	};
}

class DX11Renderer final : public Riemann::Renderer
{
public:
	DX11Renderer() {}

	~DX11Renderer() override
	{
		m_AllMesh.clear();

		if (m_pImmediateContext)
		{
			m_pImmediateContext->ClearState();
		}

		SafeRelease(m_pShadowSampler);
		SafeRelease(m_pShadowSRV);
		SafeRelease(m_pShadowDepthView);
		SafeRelease(m_pShadowTexture);
		SafeRelease(m_pDepthStencilState);
		SafeRelease(m_pRasterizerState);
		SafeRelease(m_pVertexLayout);
		SafeRelease(m_pShadowVertexShader);
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

		SetFillMode(false);
		SetDepthMode();
		SetupViewports();

		hr = CreateShaders(shaderPath);
		if (FAILED(hr))
		{
			return hr;
		}

		SetCamera(Riemann::CameraDesc());
		SetLight(Riemann::DirectionalLightDesc());
		return S_OK;
	}

	void Render() override
	{
		if (!m_pImmediateContext || !m_pSwapChain)
		{
			return;
		}

		RenderShadowPass();
		RenderMainPass();
		m_pSwapChain->Present(0, 0);
	}

	void SetCamera(const Riemann::CameraDesc& camera) override
	{
		m_Camera = camera;
		m_Eye = camera.Eye;
		m_View = Transform3::BuildViewMatrix_LHCoordinateSystem(camera.Eye, camera.At, camera.Up);
		const float aspect = m_Height != 0 ? m_Width / static_cast<float>(m_Height) : 1.0f;
		m_Projection = Transform3::BuildPerspectiveMatrix_LHCoordinateSystem(camera.FovY, aspect, camera.NearPlane, camera.FarPlane);
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
		if (meshDesc.Id.empty() || meshDesc.Vertices.empty() || meshDesc.Indices.empty())
		{
			return false;
		}

		DeleteMesh(meshDesc.Id.c_str());

		DX11StaticMesh mesh;
		mesh.Id = meshDesc.Id;
		mesh.WorldTransform = meshDesc.WorldTransform;
		mesh.Color = meshDesc.Color;
		mesh.IndexCount = static_cast<int>(meshDesc.Indices.size());
		mesh.IndexFormat = DXGI_FORMAT_R32_UINT;
		mesh.Topology = meshDesc.Topology == Riemann::RenderPrimitiveTopology::Lines
			? D3D11_PRIMITIVE_TOPOLOGY_LINELIST
			: D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		mesh.CastShadow = meshDesc.CastShadow && meshDesc.Topology == Riemann::RenderPrimitiveTopology::Triangles;

		HRESULT hr = S_OK;

		D3D11_BUFFER_DESC bd = {};
		bd.Usage = D3D11_USAGE_DEFAULT;
		bd.ByteWidth = static_cast<UINT>(sizeof(Vertex1) * meshDesc.Vertices.size());
		bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;

		D3D11_SUBRESOURCE_DATA initData = {};
		initData.pSysMem = meshDesc.Vertices.data();
		hr = m_pd3dDevice->CreateBuffer(&bd, &initData, &mesh.pVertexBuffer);
		if (FAILED(hr))
		{
			return false;
		}

		bd = {};
		bd.Usage = D3D11_USAGE_DEFAULT;
		bd.ByteWidth = static_cast<UINT>(sizeof(uint32_t) * meshDesc.Indices.size());
		bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
		initData = {};
		initData.pSysMem = meshDesc.Indices.data();
		hr = m_pd3dDevice->CreateBuffer(&bd, &initData, &mesh.pIndexBuffer);
		if (FAILED(hr))
		{
			return false;
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
		if (id == nullptr || vertices == nullptr || vertexCount <= 0)
		{
			return false;
		}

		for (DX11StaticMesh& mesh : m_AllMesh)
		{
			if (mesh.Id == id)
			{
				m_pImmediateContext->UpdateSubresource(mesh.pVertexBuffer, 0, nullptr, vertices, 0, 0);
				return true;
			}
		}
		return false;
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

	bool Reset() override
	{
		m_AllMesh.clear();
		return true;
	}

	void SetFillMode(bool wireframe) override
	{
		SafeRelease(m_pRasterizerState);

		D3D11_RASTERIZER_DESC desc = {};
		desc.FillMode = wireframe ? D3D11_FILL_WIREFRAME : D3D11_FILL_SOLID;
		desc.CullMode = D3D11_CULL_NONE;
		desc.DepthClipEnable = TRUE;
		desc.MultisampleEnable = FALSE;
		desc.AntialiasedLineEnable = FALSE;

		if (SUCCEEDED(m_pd3dDevice->CreateRasterizerState(&desc, &m_pRasterizerState)))
		{
			m_pImmediateContext->RSSetState(m_pRasterizerState);
		}
	}

	void SetDepthMode() override
	{
		SafeRelease(m_pDepthStencilState);

		D3D11_DEPTH_STENCIL_DESC desc = {};
		desc.DepthEnable = TRUE;
		desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
		desc.DepthFunc = D3D11_COMPARISON_LESS;
		desc.StencilEnable = FALSE;

		if (SUCCEEDED(m_pd3dDevice->CreateDepthStencilState(&desc, &m_pDepthStencilState)))
		{
			m_pImmediateContext->OMSetDepthStencilState(m_pDepthStencilState, 0);
		}
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
		depthTextureDesc.Format = DXGI_FORMAT_D32_FLOAT;
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
		shaderFile += L"simple.fxh";

		ID3DBlob* pVSBlob = nullptr;
		HRESULT hr = CompileShaderFromFile(shaderFile.c_str(), "VS", "vs_4_0", &pVSBlob);
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

		ID3DBlob* pPSBlob = nullptr;
		hr = CompileShaderFromFile(shaderFile.c_str(), "PS", "ps_4_0", &pPSBlob);
		if (FAILED(hr))
		{
			MessageBoxA(nullptr, "Compile PS Error", "Renderer", MB_OK);
			return hr;
		}

		hr = m_pd3dDevice->CreatePixelShader(pPSBlob->GetBufferPointer(), pPSBlob->GetBufferSize(), nullptr, &m_pPixelShader);
		pPSBlob->Release();
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

	void RenderShadowPass()
	{
		ID3D11ShaderResourceView* nullSRV[] = { nullptr };
		m_pImmediateContext->PSSetShaderResources(0, 1, nullSRV);

		m_pImmediateContext->RSSetState(m_pRasterizerState);
		m_pImmediateContext->RSSetViewports(1, &m_ShadowViewport);
		m_pImmediateContext->OMSetRenderTargets(0, nullptr, m_pShadowDepthView);
		m_pImmediateContext->ClearDepthStencilView(m_pShadowDepthView, D3D11_CLEAR_DEPTH, 1.0f, 0);

		m_pImmediateContext->IASetInputLayout(m_pVertexLayout);
		m_pImmediateContext->VSSetShader(m_pShadowVertexShader, nullptr, 0);
		m_pImmediateContext->PSSetShader(nullptr, nullptr, 0);

		DrawScene(true);
	}

	void RenderMainPass()
	{
		const float clearColor[4] = { 0.015f, 0.018f, 0.024f, 1.0f };
		m_pImmediateContext->RSSetState(m_pRasterizerState);
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

		ID3D11ShaderResourceView* nullSRV[] = { nullptr };
		m_pImmediateContext->PSSetShaderResources(0, 1, nullSRV);
	}

	void DrawScene(bool shadowPass)
	{
		for (DX11StaticMesh& mesh : m_AllMesh)
		{
			if (shadowPass && !mesh.CastShadow)
			{
				continue;
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

			m_pImmediateContext->UpdateSubresource(mesh.pConstantBuffer, 0, nullptr, &cb, 0, 0);
			m_pImmediateContext->VSSetConstantBuffers(0, 1, &mesh.pConstantBuffer);
			m_pImmediateContext->PSSetConstantBuffers(0, 1, &mesh.pConstantBuffer);

			UINT stride = sizeof(Vertex1);
			UINT offset = 0;
			m_pImmediateContext->IASetVertexBuffers(0, 1, &mesh.pVertexBuffer, &stride, &offset);
			m_pImmediateContext->IASetIndexBuffer(mesh.pIndexBuffer, mesh.IndexFormat, 0);
			m_pImmediateContext->IASetPrimitiveTopology(mesh.Topology);
			m_pImmediateContext->DrawIndexed(mesh.IndexCount, 0, 0);
		}
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
	ID3D11RenderTargetView* m_pRenderTargetView = nullptr;
	ID3D11DepthStencilView* m_pDepthBuffer = nullptr;
	ID3D11VertexShader* m_pVertexShader = nullptr;
	ID3D11VertexShader* m_pShadowVertexShader = nullptr;
	ID3D11PixelShader* m_pPixelShader = nullptr;
	ID3D11InputLayout* m_pVertexLayout = nullptr;
	ID3D11RasterizerState* m_pRasterizerState = nullptr;
	ID3D11DepthStencilState* m_pDepthStencilState = nullptr;
	ID3D11Texture2D* m_pShadowTexture = nullptr;
	ID3D11DepthStencilView* m_pShadowDepthView = nullptr;
	ID3D11ShaderResourceView* m_pShadowSRV = nullptr;
	ID3D11SamplerState* m_pShadowSampler = nullptr;

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
