
#include "DX11Renderer.h"

#include <string>
#include <vector>
#include <d3d11_1.h>
#include <d3dcompiler.h>
#include <directxmath.h>
#include <directxcolors.h>

#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/AxisAlignedBox3d.h"
#include "../Src/CollisionPrimitive/Sphere3d.h"
#include "../Src/CollisionPrimitive/Plane3d.h"
#include "../Src/CollisionPrimitive/Cylinder3d.h"
#include "../Src/CollisionPrimitive/Capsule3d.h"
#include "../Src/CollisionPrimitive/Mesh.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/CollisionPrimitive/HeightField3d.h"
#include "../Src/Maths/Transform.h"

using namespace DirectX;

//--------------------------------------------------------------------------------------
// Structures
//--------------------------------------------------------------------------------------

struct ConstantBuffer
{
    Matrix4d World;
    Matrix4d View;
    Matrix4d Projection;
};

struct DX11StaticMesh
{
    std::string   Id;
    ID3D11Buffer* pVertexBuffer = nullptr;
    ID3D11Buffer* pIndexBuffer = nullptr;
    ID3D11Buffer* pConstantBuffer = nullptr;
    Transform*    Trans;
    int           IndexCount = 0;
    DXGI_FORMAT   IndexFormat = DXGI_FORMAT_R32_UINT;
    D3D11_PRIMITIVE_TOPOLOGY   Topology;

    void Release()
    {
        if (pConstantBuffer) pConstantBuffer->Release();
        if (pVertexBuffer) pVertexBuffer->Release();
        if (pIndexBuffer) pIndexBuffer->Release();
    }
};

class DX11Renderer : public Renderer
{
public:
    DX11Renderer()
    {

    }

    //--------------------------------------------------------------------------------------
    // Clean up the objects we've created
    //--------------------------------------------------------------------------------------
    virtual ~DX11Renderer() override
    {
        for (size_t i = 0; i < m_AllMesh.size(); ++i)
        {
            m_AllMesh[i].Release();
        }

        if (m_pImmediateContext) m_pImmediateContext->ClearState();
        if (m_pVertexLayout) m_pVertexLayout->Release();
        if (m_pVertexShader) m_pVertexShader->Release();
        if (m_pPixelShader) m_pPixelShader->Release();
        if (m_pRenderTargetView) m_pRenderTargetView->Release();
        if (m_pSwapChain1) m_pSwapChain1->Release();
        if (m_pSwapChain) m_pSwapChain->Release();
        if (m_pImmediateContext1) m_pImmediateContext1->Release();
        if (m_pImmediateContext) m_pImmediateContext->Release();
        if (m_pd3dDevice1) m_pd3dDevice1->Release();
        if (m_pd3dDevice) m_pd3dDevice->Release();
    }

    //--------------------------------------------------------------------------------------
    // Create Direct3D device and swap chain
    //--------------------------------------------------------------------------------------
    HRESULT InitDevice(HWND hWnd, const char* shader_path)
    {
        HRESULT hr = S_OK;

        RECT rc;
        GetClientRect(hWnd, &rc);
        UINT width = rc.right - rc.left;
        UINT height = rc.bottom - rc.top;

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
        UINT numDriverTypes = ARRAYSIZE(driverTypes);

        D3D_FEATURE_LEVEL featureLevels[] =
        {
            D3D_FEATURE_LEVEL_11_1,
            D3D_FEATURE_LEVEL_11_0,
            D3D_FEATURE_LEVEL_10_1,
            D3D_FEATURE_LEVEL_10_0,
        };
        UINT numFeatureLevels = ARRAYSIZE(featureLevels);

        for (UINT driverTypeIndex = 0; driverTypeIndex < numDriverTypes; driverTypeIndex++)
        {
            m_driverType = driverTypes[driverTypeIndex];
            hr = D3D11CreateDevice(nullptr, m_driverType, nullptr, createDeviceFlags, featureLevels, numFeatureLevels,
                D3D11_SDK_VERSION, &m_pd3dDevice, &m_featureLevel, &m_pImmediateContext);

            if (hr == E_INVALIDARG)
            {
                // DirectX 11.0 platforms will not recognize D3D_FEATURE_LEVEL_11_1 so we need to retry without it
                hr = D3D11CreateDevice(nullptr, m_driverType, nullptr, createDeviceFlags, &featureLevels[1], numFeatureLevels - 1,
                    D3D11_SDK_VERSION, &m_pd3dDevice, &m_featureLevel, &m_pImmediateContext);
            }

            if (SUCCEEDED(hr))
                break;
        }
        if (FAILED(hr))
            return hr;

        // Obtain DXGI factory from device (since we used nullptr for pAdapter above)
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
            return hr;

        // Create swap chain
        IDXGIFactory2* dxgiFactory2 = nullptr;
        hr = dxgiFactory->QueryInterface(__uuidof(IDXGIFactory2), reinterpret_cast<void**>(&dxgiFactory2));
        if (dxgiFactory2)
        {
            // DirectX 11.1 or later
            hr = m_pd3dDevice->QueryInterface(__uuidof(ID3D11Device1), reinterpret_cast<void**>(&m_pd3dDevice1));
            if (SUCCEEDED(hr))
            {
                (void)m_pImmediateContext->QueryInterface(__uuidof(ID3D11DeviceContext1), reinterpret_cast<void**>(&m_pImmediateContext1));
            }

            DXGI_SWAP_CHAIN_DESC1 sd = {};
            sd.Width = width;
            sd.Height = height;
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
            // DirectX 11.0 systems
            DXGI_SWAP_CHAIN_DESC sd = {};
            sd.BufferCount = 1;
            sd.BufferDesc.Width = width;
            sd.BufferDesc.Height = height;
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

        // Note this tutorial doesn't handle full-screen swapchains so we block the ALT+ENTER shortcut
        dxgiFactory->MakeWindowAssociation(hWnd, DXGI_MWA_NO_ALT_ENTER);

        dxgiFactory->Release();

        if (FAILED(hr))
            return hr;

        // Create a render target view
        ID3D11Texture2D* pBackBuffer = nullptr;
        hr = m_pSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&pBackBuffer));
        if (FAILED(hr))
            return hr;

        hr = m_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &m_pRenderTargetView);
        pBackBuffer->Release();
        if (FAILED(hr))
            return hr;

        D3D11_TEXTURE2D_DESC depthTextureDesc;
        ZeroMemory(&depthTextureDesc, sizeof(depthTextureDesc));
        depthTextureDesc.Width = width;
        depthTextureDesc.Height = height;
        depthTextureDesc.MipLevels = 1;
        depthTextureDesc.ArraySize = 1;
        depthTextureDesc.SampleDesc.Count = 1;
        depthTextureDesc.Format = DXGI_FORMAT_D32_FLOAT;
        depthTextureDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;

        ID3D11Texture2D* DepthStencilTexture;
        hr = m_pd3dDevice->CreateTexture2D(&depthTextureDesc, nullptr, &DepthStencilTexture);
        if (FAILED(hr))
            return hr;

        D3D11_DEPTH_STENCIL_VIEW_DESC dsvDesc;
        ZeroMemory(&dsvDesc, sizeof(dsvDesc));
        dsvDesc.Format = depthTextureDesc.Format;
        dsvDesc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2DMS;

        hr = m_pd3dDevice->CreateDepthStencilView(DepthStencilTexture, &dsvDesc, &m_pDepthBuffer);
        DepthStencilTexture->Release();
        if (FAILED(hr))
            return hr;

        m_pImmediateContext->OMSetRenderTargets(1, &m_pRenderTargetView, m_pDepthBuffer);

        SetFillMode(false);
        SetDepthMode();

        // Setup the viewport
        D3D11_VIEWPORT vp;
        vp.Width = (FLOAT)width;
        vp.Height = (FLOAT)height;
        vp.MinDepth = 0.0f;
        vp.MaxDepth = 1.0f;
        vp.TopLeftX = 0;
        vp.TopLeftY = 0;
        m_pImmediateContext->RSSetViewports(1, &vp);

        // Compile the vertex shader
        ID3DBlob* pVSBlob = nullptr;
        std::wstring shader_file;
        size_t n = strlen(shader_path);
        for (size_t i = 0; i < n; ++i)
        {
            shader_file += wchar_t(shader_path[i]);
        }
        shader_file += L"/simple.fxh";

        hr = CompileShaderFromFile(shader_file.c_str(), "VS", "vs_4_0", &pVSBlob);
        if (FAILED(hr))
        {
			const char* str = pVSBlob ? (const char*)pVSBlob->GetBufferPointer() : "Compile VS Error";
            MessageBoxA(nullptr, str, "Error", MB_OK);
            return hr;
        }

        // Create the vertex shader
        hr = m_pd3dDevice->CreateVertexShader(pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), nullptr, &m_pVertexShader);
        if (FAILED(hr))
        {
            pVSBlob->Release();
            return hr;
        }

        // Define the input layout
        D3D11_INPUT_ELEMENT_DESC layout[] =
        {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
            { "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
        };
        UINT numElements = ARRAYSIZE(layout);

        // Create the input layout
        hr = m_pd3dDevice->CreateInputLayout(layout, numElements, pVSBlob->GetBufferPointer(),
            pVSBlob->GetBufferSize(), &m_pVertexLayout);
        pVSBlob->Release();
        if (FAILED(hr))
            return hr;

        // Set the input layout
        m_pImmediateContext->IASetInputLayout(m_pVertexLayout);

        // Compile the pixel shader
        ID3DBlob* pPSBlob = nullptr;
        hr = CompileShaderFromFile(shader_file.c_str(), "PS", "ps_4_0", &pPSBlob);
        if (FAILED(hr))
        {
			const char* str = pVSBlob ? (const char*)pVSBlob->GetBufferPointer() : "Compile PS Error";
			MessageBoxA(nullptr, str, "Error", MB_OK);
			return hr;
        }

        // Create the pixel shader
        hr = m_pd3dDevice->CreatePixelShader(pPSBlob->GetBufferPointer(), pPSBlob->GetBufferSize(), nullptr, &m_pPixelShader);
        pPSBlob->Release();
        if (FAILED(hr))
            return hr;

        // Initialize the view matrix
        SetCameraLookAt(Vector3d(0.0f, 0.0f, 5.0f), Vector3d(0.0f, 0.0f, 0.0f));

        // Initialize the projection matrix
        m_Projection = Transform::BuildPerspectiveMatrix_LHCoordinateSystem(XM_PIDIV2, width / (FLOAT)height, 0.001f, 100000.0f);
        return S_OK;
    }

    virtual void SetFillMode(bool Wireframe) override
    {
        D3D11_RASTERIZER_DESC wfdesc;
        ZeroMemory(&wfdesc, sizeof(D3D11_RASTERIZER_DESC));
        wfdesc.FillMode = Wireframe ? D3D11_FILL_WIREFRAME : D3D11_FILL_SOLID;
        wfdesc.CullMode = D3D11_CULL_NONE;

        ID3D11RasterizerState* pWireFrame = nullptr;
        HRESULT hr = m_pd3dDevice->CreateRasterizerState(&wfdesc, &pWireFrame);
        if (FAILED(hr))
            return;

        m_pImmediateContext->RSSetState(pWireFrame);
    }

    virtual void SetDepthMode() override
    {
        D3D11_DEPTH_STENCIL_DESC depthstencilDesc;
        ZeroMemory(&depthstencilDesc, sizeof(depthstencilDesc));
        depthstencilDesc.DepthEnable = TRUE;
        depthstencilDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
        depthstencilDesc.DepthFunc = D3D11_COMPARISON_LESS;
        depthstencilDesc.StencilEnable = FALSE;

        ID3D11DepthStencilState* pDepthStencilState = nullptr;
        HRESULT hr = m_pd3dDevice->CreateDepthStencilState(&depthstencilDesc, &pDepthStencilState);
        if (FAILED(hr))
            return;
        m_pImmediateContext->OMSetDepthStencilState(pDepthStencilState, 0);
    }

    virtual void SetCameraLookAt(Vector3d Eye, Vector3d At) override
    {
        Vector3d Up = Vector3d(0.0f, 1.0f, 0.0f);
        m_View = Transform::BuildViewMatrix_LHCoordinateSystem(Eye, At, Up);
    }

    virtual bool AddTriangles(const char* Id, Transform* pTrans, const Vertex1* pVerties, int nVerties, const void* pIndices, int nIndices, int IndicesWidth) override
    {
        return AddPrimitive(Id, pTrans, pVerties, nVerties, pIndices, nIndices, IndicesWidth, D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    }

	virtual bool AddWireframe(const char* Id, Transform* pTrans, const Vertex1* pVerties, int nVerties, const void* pIndices, int nIndices) override
	{
        return AddPrimitive(Id, pTrans, pVerties, nVerties, pIndices, nIndices, 2, D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
	}

    bool AddPrimitive(const char* Id, Transform *pTrans, const Vertex1* pVerties, int nVerties, const void* pIndices, int nIndices, int IndicesWidth, D3D11_PRIMITIVE_TOPOLOGY Topology)
    {
        HRESULT hr = S_OK;

        DX11StaticMesh mesh;
        mesh.Id = std::string(Id);

        // Create vertex buffer
        D3D11_BUFFER_DESC bd = {};
        bd.Usage = D3D11_USAGE_DEFAULT;
        bd.ByteWidth = sizeof(Vertex1) * nVerties;
        bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        bd.CPUAccessFlags = 0;

        D3D11_SUBRESOURCE_DATA InitData = {};
        InitData.pSysMem = (const void*)pVerties;
        hr = m_pd3dDevice->CreateBuffer(&bd, &InitData, &mesh.pVertexBuffer);
        if (FAILED(hr))
            return false;

        // Set vertex buffer
        UINT stride = sizeof(Vertex1);
        UINT offset = 0;
        m_pImmediateContext->IASetVertexBuffers(0, 1, &mesh.pVertexBuffer, &stride, &offset);

        // Create index buffer
        bd.Usage = D3D11_USAGE_DEFAULT;
        bd.ByteWidth = IndicesWidth * nIndices;
        bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
        bd.CPUAccessFlags = 0;
        InitData.pSysMem = pIndices;
        hr = m_pd3dDevice->CreateBuffer(&bd, &InitData, &mesh.pIndexBuffer);
        if (FAILED(hr))
            return false;

        // Set index buffer
        mesh.IndexFormat = IndicesWidth == 4 ? DXGI_FORMAT_R32_UINT : DXGI_FORMAT_R16_UINT;
        m_pImmediateContext->IASetIndexBuffer(mesh.pIndexBuffer, mesh.IndexFormat, 0);


        // Create the constant buffer
        bd.Usage = D3D11_USAGE_DEFAULT;
        bd.ByteWidth = sizeof(ConstantBuffer);
        bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        bd.CPUAccessFlags = 0;
        hr = m_pd3dDevice->CreateBuffer(&bd, nullptr, &mesh.pConstantBuffer);
        if (FAILED(hr))
            return false;

        // Set primitive topology
        mesh.Topology = Topology;
        // Initialize the world matrix
        mesh.Trans = pTrans;
        mesh.IndexCount = nIndices;

        m_AllMesh.push_back(mesh);

        return true;
    }

    virtual bool UpdateVerties(const char* Id, const Vertex1* pVerties, int nVerties)
    {
        for (size_t i = 0; i < m_AllMesh.size(); ++i)
        {
            DX11StaticMesh& mesh = m_AllMesh[i];
            if (strcmp(Id, mesh.Id.c_str()) == 0)
            {
                m_pImmediateContext->UpdateSubresource(mesh.pVertexBuffer, 0, nullptr, pVerties, 0, 0);
                return true;
            }
        }
        return false;
    }

    virtual bool DeleteMesh(const char* Id) override
    {
        for (size_t i = 0; i < m_AllMesh.size(); ++i)
        {
            DX11StaticMesh& mesh = m_AllMesh[i];
            if (strcmp(Id, mesh.Id.c_str()) == 0)
            {
                mesh.Release();
                m_AllMesh.erase(m_AllMesh.begin() + i);
                return true;
            }
        }
        return false;
    }

    template <class TShape>
    void AddGeometry(Geometry* geom, bool DrawMesh = true)
    {
		std::vector<Vector3d> Vertices;
		std::vector<uint16_t> Indices;
        std::vector<Vector3d> Normals;
        TShape* shape = geom->CastGeometry<TShape>();

        if (DrawMesh)
        {
			shape->GetMesh(Vertices, Indices, Normals);
			std::vector<Vertex1> vv;
			for (size_t i = 0; i < Vertices.size(); ++i)
			{
				vv.emplace_back(Vertices[i], Normals[i]);
			}
			AddTriangles(geom->GetName(), geom->GetTransform(), &vv[0], (int)vv.size(), &Indices[0], (int)Indices.size(), 2);
        }
        else
        {
			shape->GetWireframe(Vertices, Indices);
			std::vector<Vertex1> vv;
			for (size_t i = 0; i < Vertices.size(); ++i)
			{
				vv.emplace_back(Vertices[i], Vector3d(1.0f, 1.0f, 1.0f));
			}
			AddWireframe(geom->GetName(), geom->GetTransform(), &vv[0], (int)vv.size(), &Indices[0], (int)Indices.size());
        }
    }

	virtual void AddTriMesh(Mesh* mesh, Transform* Trans, bool RenderBV) override
	{
		mesh->CalculateNormals();

		std::vector<Vertex1> vv;
		for (uint32_t i = 0; i < mesh->GetNumVertices(); ++i)
		{
			vv.emplace_back(mesh->Vertices[i], mesh->Normals[i]);
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

    virtual void AddGeometry(Geometry* geom) override
	{
		if (geom->GetShapeType() == TRIANGLE_MESH)
		{
			AddTriMesh(geom->CastGeometry<Mesh>(), geom->GetTransform(), true);
		}
		else if (geom->GetShapeType() == CONVEX_MESH)
		{
            AddGeometry<ConvexMesh>(geom);
		}
        else if (geom->GetShapeType() == OBB)
        {
            // AddGeometry<AxisAlignedBox3d>(geom);
        }
		else if (geom->GetShapeType() == PLANE)
		{
            AddGeometry <Plane3d > (geom);
		}
		else if (geom->GetShapeType() == SPHERE)
		{
			AddGeometry <Sphere3d >(geom);
		}
		else if (geom->GetShapeType() == CAPSULE)
		{
			AddGeometry <Capsule3d >(geom);
		}
		else if (geom->GetShapeType() == CYLINDER)
		{
			AddGeometry <Cylinder3d >(geom);
		}
		else if (geom->GetShapeType() == HEIGHTFIELD)
		{
			AddGeometry <HeightField3d >(geom);
		}
	}

    //--------------------------------------------------------------------------------------
    // Helper for compiling shaders with D3DCompile
    //
    // With VS 11, we could load up prebuilt .cso files instead...
    //--------------------------------------------------------------------------------------
    static HRESULT CompileShaderFromFile(const WCHAR* szFileName, LPCSTR szEntryPoint, LPCSTR szShaderModel, ID3DBlob** ppBlobOut)
    {
        HRESULT hr = S_OK;

        DWORD dwShaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
#ifdef _DEBUG
        // Set the D3DCOMPILE_DEBUG flag to embed debug information in the shaders.
        // Setting this flag improves the shader debugging experience, but still allows 
        // the shaders to be optimized and to run exactly the way they will run in 
        // the release configuration of this program.
        dwShaderFlags |= D3DCOMPILE_DEBUG;

        // Disable optimizations to further improve shader debugging
        dwShaderFlags |= D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

        ID3DBlob* pErrorBlob = nullptr;
        hr = D3DCompileFromFile(szFileName, nullptr, nullptr, szEntryPoint, szShaderModel, dwShaderFlags, 0, ppBlobOut, &pErrorBlob);
        if (FAILED(hr))
        {
            if (pErrorBlob)
            {
                OutputDebugStringA(reinterpret_cast<const char*>(pErrorBlob->GetBufferPointer()));
                pErrorBlob->Release();
            }
            return hr;
        }
        if (pErrorBlob) pErrorBlob->Release();

        return S_OK;
    }

    //--------------------------------------------------------------------------------------
    // Render a frame
    //--------------------------------------------------------------------------------------
    virtual void Render() override
    {
        // Clear the back buffer
        m_pImmediateContext->ClearRenderTargetView(m_pRenderTargetView, Colors::Black);
        m_pImmediateContext->ClearDepthStencilView(m_pDepthBuffer, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

        // Set VS and PS
        m_pImmediateContext->VSSetShader(m_pVertexShader, nullptr, 0);
        m_pImmediateContext->PSSetShader(m_pPixelShader, nullptr, 0);

        // Update variables
        ConstantBuffer cb;
        cb.View = m_View;
        cb.Projection = m_Projection;

        for (size_t i = 0; i < m_AllMesh.size(); ++i)
        {
            DX11StaticMesh& mesh = m_AllMesh[i];

            cb.World = mesh.Trans->GetWorldMatrix();
            m_pImmediateContext->UpdateSubresource(mesh.pConstantBuffer, 0, nullptr, &cb, 0, 0);
            m_pImmediateContext->VSSetConstantBuffers(0, 1, &mesh.pConstantBuffer);

            UINT stride = sizeof(Vertex1);
            UINT offset = 0;
            m_pImmediateContext->IASetVertexBuffers(0, 1, &mesh.pVertexBuffer, &stride, &offset);
            m_pImmediateContext->IASetIndexBuffer(mesh.pIndexBuffer, mesh.IndexFormat, 0);
			m_pImmediateContext->IASetPrimitiveTopology(mesh.Topology);
            m_pImmediateContext->DrawIndexed(mesh.IndexCount, 0, 0);
        }

        // Present our back buffer to our front buffer
        m_pSwapChain->Present(0, 0);
    }

private:
    D3D_DRIVER_TYPE         m_driverType = D3D_DRIVER_TYPE_NULL;
    D3D_FEATURE_LEVEL       m_featureLevel = D3D_FEATURE_LEVEL_11_0;
    ID3D11Device* m_pd3dDevice = nullptr;
    ID3D11Device1* m_pd3dDevice1 = nullptr;
    ID3D11DeviceContext* m_pImmediateContext = nullptr;
    ID3D11DeviceContext1* m_pImmediateContext1 = nullptr;
    IDXGISwapChain* m_pSwapChain = nullptr;
    IDXGISwapChain1* m_pSwapChain1 = nullptr;
    ID3D11RenderTargetView* m_pRenderTargetView = nullptr;
    ID3D11VertexShader* m_pVertexShader = nullptr;
    ID3D11PixelShader* m_pPixelShader = nullptr;
    ID3D11InputLayout* m_pVertexLayout = nullptr;
    ID3D11DepthStencilView* m_pDepthBuffer = nullptr;

    Matrix4d                m_View;
    Matrix4d                m_Projection;

    std::vector<DX11StaticMesh> m_AllMesh;
};

// static
Renderer* Renderer::CreateDX11Renderer(void* hWnd, const char* shader_path)
{
    DX11Renderer* p = new DX11Renderer();
    if (p->InitDevice((HWND)hWnd, shader_path) == S_OK)
    {
        return p;
    }
    delete p;
    return nullptr;
}