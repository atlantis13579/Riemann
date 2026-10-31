#pragma once

#include <algorithm>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>
#include "RiemannRenderer.h"

namespace Riemann
{
	// Uploads a reusable R32F 3D density texture and integrates gas along each
	// view ray. Both grid and SPH representations use this path. Opaque scene
	// depth clips the ray; the viewer supplies compatible D24S8 DSV / R24 SRV
	// views and unbinds the depth target before sampling. Reset and resize
	// release or recreate the corresponding GPU resources. DX11 owns this path;
	// renderers without volume support keep their isosurface visualization.
	// References: NVIDIA GPU Gems 3, chapter 30, and GPU Gems, chapter 39:
	// https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-30-real-time-simulation-and-rendering-3d-fluids
	// https://developer.nvidia.com/gpugems/gpugems/part-vi-beyond-triangles/chapter-39-volume-rendering-techniques
	class DX11VolumeRenderer
	{
	public:
		bool Initialize(ID3D11Device* Device, const wchar_t* ShaderFile)
		{
			using Microsoft::WRL::ComPtr;
			ComPtr<ID3DBlob> Code, Errors;
			if (FAILED(D3DCompileFromFile(ShaderFile, nullptr, nullptr, "VolumeVS", "vs_4_0",
				D3DCOMPILE_ENABLE_STRICTNESS, 0, &Code, &Errors))) return false;
			if (FAILED(Device->CreateVertexShader(Code->GetBufferPointer(), Code->GetBufferSize(), nullptr, &m_VS))) return false;
			Code.Reset(); Errors.Reset();
			if (FAILED(D3DCompileFromFile(ShaderFile, nullptr, nullptr, "VolumePS", "ps_4_0",
				D3DCOMPILE_ENABLE_STRICTNESS, 0, &Code, &Errors))) return false;
			if (FAILED(Device->CreatePixelShader(Code->GetBufferPointer(), Code->GetBufferSize(), nullptr, &m_PS))) return false;
			D3D11_BUFFER_DESC Buffer = {};
			Buffer.ByteWidth = sizeof(Parameters);
			Buffer.Usage = D3D11_USAGE_DEFAULT;
			Buffer.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
			if (FAILED(Device->CreateBuffer(&Buffer, nullptr, &m_Constants))) return false;
			D3D11_SAMPLER_DESC Sampler = {};
			Sampler.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
			Sampler.AddressU = Sampler.AddressV = Sampler.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
			Sampler.MaxLOD = D3D11_FLOAT32_MAX;
			if (FAILED(Device->CreateSamplerState(&Sampler, &m_Sampler))) return false;
			D3D11_BLEND_DESC Blend = {};
			Blend.RenderTarget[0].BlendEnable = TRUE;
			Blend.RenderTarget[0].SrcBlend = Blend.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
			Blend.RenderTarget[0].DestBlend = Blend.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
			Blend.RenderTarget[0].BlendOp = Blend.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
			Blend.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
			if (FAILED(Device->CreateBlendState(&Blend, &m_Blend))) return false;
			D3D11_DEPTH_STENCIL_DESC Depth = {};
			Depth.DepthEnable = FALSE;
			if (FAILED(Device->CreateDepthStencilState(&Depth, &m_Depth))) return false;
			D3D11_RASTERIZER_DESC Raster = {};
			Raster.FillMode = D3D11_FILL_SOLID; Raster.CullMode = D3D11_CULL_NONE;
			Raster.DepthClipEnable = TRUE;
			return SUCCEEDED(Device->CreateRasterizerState(&Raster, &m_Raster));
		}

		void Clear() { m_Texture.Reset(); m_DensityView.Reset(); m_Description.Density.clear(); }

		bool Update(ID3D11Device* Device, ID3D11DeviceContext* Context, const DensityVolumeDesc& Volume)
		{
			if (!Volume.IsValid()) return false;
			if (!m_Texture || Volume.Resolution != m_Description.Resolution)
			{
				D3D11_TEXTURE3D_DESC Texture = {};
				Texture.Width = Volume.Resolution.x; Texture.Height = Volume.Resolution.y; Texture.Depth = Volume.Resolution.z;
				Texture.MipLevels = 1; Texture.Format = DXGI_FORMAT_R32_FLOAT;
				Texture.Usage = D3D11_USAGE_DEFAULT; Texture.BindFlags = D3D11_BIND_SHADER_RESOURCE;
				Microsoft::WRL::ComPtr<ID3D11Texture3D> NewTexture;
				Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> NewView;
				if (FAILED(Device->CreateTexture3D(&Texture, nullptr, &NewTexture))
					|| FAILED(Device->CreateShaderResourceView(NewTexture.Get(), nullptr, &NewView))) return false;
				m_Texture = NewTexture; m_DensityView = NewView;
			}
			Context->UpdateSubresource(m_Texture.Get(), 0, nullptr, Volume.Density.data(),
				Volume.Resolution.x * sizeof(float), Volume.Resolution.x * Volume.Resolution.y * sizeof(float));
			m_Description = Volume;
			return true;
		}

		void Render(ID3D11DeviceContext* Context, ID3D11RenderTargetView* Target,
			ID3D11ShaderResourceView* SceneDepth, const CameraDesc& Camera,
			const DirectionalLightDesc& Light, unsigned Width, unsigned Height)
		{
			if (!m_Texture || !SceneDepth || Width == 0 || Height == 0) return;
			const Vector3 Forward = Unit(Camera.At - Camera.Eye, Vector3::UnitZ());
			const Vector3 Right = Unit(Camera.Up.Cross(Forward), Vector3::UnitX());
			const Vector3 Up = Forward.Cross(Right);
			const Vector3 Extent = m_Description.BoundsMax - m_Description.BoundsMin;
			const float Cell = std::min(Extent.x / m_Description.Resolution.x,
				std::min(Extent.y / m_Description.Resolution.y, Extent.z / m_Description.Resolution.z));
			const float TanHalfFov = std::tan(0.5f * Camera.FovY);
			Parameters P;
			P.EyeNear = V4(Camera.Eye, Camera.NearPlane);
			P.ForwardFar = V4(Forward, Camera.FarPlane);
			P.Right = V4(Right * (TanHalfFov * Width / Height), 0);
			P.Up = V4(Up * TanHalfFov, 0);
			P.MinDensity = V4(m_Description.BoundsMin, 1.0f / m_Description.AmbientDensity);
			P.MaxThreshold = V4(m_Description.BoundsMax, m_Description.RelativeThreshold);
			P.Compressed = m_Description.CompressedColor; P.Rarefied = m_Description.RarefiedColor;
			P.Light = V4(Unit(Light.Direction, Vector3(0, -1, 0)), std::max(0.2f, Light.Ambient));
			P.Control = Vector4(Cell * m_Description.StepScale, m_Description.Extinction, (float)Width, (float)Height);
			P.Smoke = Vector4(m_Description.Smoke ? 1.0f : 0.0f, m_Description.ShadowStep, 0, 0);
			Context->UpdateSubresource(m_Constants.Get(), 0, nullptr, &P, 0, 0);
			ID3D11Buffer* Constants = m_Constants.Get();
			Context->PSSetConstantBuffers(1, 1, &Constants);
			Context->OMSetRenderTargets(1, &Target, nullptr);
			Context->OMSetDepthStencilState(m_Depth.Get(), 0);
			const float Factor[4] = {};
			Context->OMSetBlendState(m_Blend.Get(), Factor, 0xffffffffu);
			Context->RSSetState(m_Raster.Get());
			Context->IASetInputLayout(nullptr);
			Context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
			Context->VSSetShader(m_VS.Get(), nullptr, 0); Context->PSSetShader(m_PS.Get(), nullptr, 0);
			ID3D11ShaderResourceView* Views[2] = { m_DensityView.Get(), SceneDepth };
			Context->PSSetShaderResources(1, 2, Views);
			ID3D11SamplerState* Sampler = m_Sampler.Get();
			Context->PSSetSamplers(1, 1, &Sampler);
			Context->Draw(3, 0);
			Views[0] = Views[1] = nullptr;
			Context->PSSetShaderResources(1, 2, Views);
			Context->OMSetBlendState(nullptr, Factor, 0xffffffffu);
		}

	private:
		static Vector3 Unit(const Vector3& V, const Vector3& Fallback)
		{ return V.SquareLength() > 1.0e-8f ? V / V.Length() : Fallback; }
		static Vector4 V4(const Vector3& V, float W) { return Vector4(V.x, V.y, V.z, W); }
		struct Parameters
		{
			Vector4 EyeNear, ForwardFar, Right, Up, MinDensity, MaxThreshold;
			Vector4 Compressed, Rarefied, Light, Control, Smoke;
		};
		DensityVolumeDesc m_Description;
		Microsoft::WRL::ComPtr<ID3D11VertexShader> m_VS;
		Microsoft::WRL::ComPtr<ID3D11PixelShader> m_PS;
		Microsoft::WRL::ComPtr<ID3D11Buffer> m_Constants;
		Microsoft::WRL::ComPtr<ID3D11Texture3D> m_Texture;
		Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> m_DensityView;
		Microsoft::WRL::ComPtr<ID3D11SamplerState> m_Sampler;
		Microsoft::WRL::ComPtr<ID3D11BlendState> m_Blend;
		Microsoft::WRL::ComPtr<ID3D11DepthStencilState> m_Depth;
		Microsoft::WRL::ComPtr<ID3D11RasterizerState> m_Raster;
	};
}
