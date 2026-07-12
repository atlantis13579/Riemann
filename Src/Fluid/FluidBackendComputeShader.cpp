#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)

#include "FluidBackend.h"

#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <d3d11.h>
#include <d3dcompiler.h>
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "d3dcompiler.lib")
#endif

namespace Riemann
{
#if defined(_WIN32)
	namespace
	{
		template <class T> void SafeRelease(T*& Object)
		{
			if (Object) { Object->Release(); Object = nullptr; }
		}

		const char* ShaderSource = R"HLSL(
ByteAddressBuffer InputData : register(t0);
RWByteAddressBuffer OutputData : register(u0);
cbuffer Constants : register(b0) { uint VectorCount; uint3 Padding; };
[numthreads(256, 1, 1)]
void ApplyVelocityDelta(uint3 Id : SV_DispatchThreadID)
{
	if (Id.x >= VectorCount) return;
	uint Base = Id.x * 12;
	uint DeltaBase = VectorCount * 12 + Base;
	OutputData.Store(Base + 0, asuint(asfloat(InputData.Load(Base + 0)) + asfloat(InputData.Load(DeltaBase + 0))));
	OutputData.Store(Base + 4, asuint(asfloat(InputData.Load(Base + 4)) + asfloat(InputData.Load(DeltaBase + 4))));
	OutputData.Store(Base + 8, asuint(asfloat(InputData.Load(Base + 8)) + asfloat(InputData.Load(DeltaBase + 8))));
}
)HLSL";

		struct Context
		{
			ID3D11Device* Device = nullptr;
			ID3D11DeviceContext* DeviceContext = nullptr;
			ID3D11ComputeShader* Shader = nullptr;
			std::mutex Mutex;
			bool Attempted = false;
			bool Available = false;

			~Context()
			{
				SafeRelease(Shader);
				SafeRelease(DeviceContext);
				SafeRelease(Device);
			}

			bool Initialize()
			{
				if (Attempted) return Available;
				Attempted = true;
				D3D_FEATURE_LEVEL Level;
				if (FAILED(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0,
					nullptr, 0, D3D11_SDK_VERSION, &Device, &Level, &DeviceContext))) return false;
				ID3DBlob* Code = nullptr;
				ID3DBlob* Errors = nullptr;
				const HRESULT Compiled = D3DCompile(ShaderSource, std::strlen(ShaderSource), nullptr,
					nullptr, nullptr, "ApplyVelocityDelta", "cs_5_0", 0, 0, &Code, &Errors);
				SafeRelease(Errors);
				if (FAILED(Compiled)) { SafeRelease(Code); return false; }
				Available = SUCCEEDED(Device->CreateComputeShader(Code->GetBufferPointer(),
					Code->GetBufferSize(), nullptr, &Shader));
				SafeRelease(Code);
				return Available;
			}
		};

		Context& GetContext() { static Context Value; return Value; }
	}

	bool IsFluidComputeShaderBackendAvailable()
	{
		Context& Value = GetContext();
		std::lock_guard<std::mutex> Lock(Value.Mutex);
		return Value.Initialize();
	}

	bool ApplyFluidVelocityDeltaComputeShader(const Vector3* Velocity,
		const Vector3* Delta, size_t Count, Vector3* Output)
	{
		if (Count == 0) return true;
		if (Count > UINT32_MAX) return false;
		Context& Value = GetContext();
		std::lock_guard<std::mutex> Lock(Value.Mutex);
		if (!Value.Initialize()) return false;
		const UINT VectorBytes = (UINT)(Count * sizeof(Vector3));
		const UINT InputBytes = VectorBytes * 2;
		D3D11_BUFFER_DESC InputDesc = {};
		InputDesc.ByteWidth = InputBytes;
		InputDesc.Usage = D3D11_USAGE_IMMUTABLE;
		InputDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		InputDesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
		std::vector<uint8_t> Input(InputBytes);
		std::memcpy(Input.data(), Velocity, VectorBytes);
		std::memcpy(Input.data() + VectorBytes, Delta, VectorBytes);
		D3D11_SUBRESOURCE_DATA Initial = { Input.data(), 0, 0 };
		ID3D11Buffer* InputBuffer = nullptr;
		if (FAILED(Value.Device->CreateBuffer(&InputDesc, &Initial, &InputBuffer))) return false;
		D3D11_SHADER_RESOURCE_VIEW_DESC SRVDesc = {};
		SRVDesc.Format = DXGI_FORMAT_R32_TYPELESS;
		SRVDesc.ViewDimension = D3D11_SRV_DIMENSION_BUFFEREX;
		SRVDesc.BufferEx.NumElements = InputBytes / 4;
		SRVDesc.BufferEx.Flags = D3D11_BUFFEREX_SRV_FLAG_RAW;
		ID3D11ShaderResourceView* SRV = nullptr;
		if (FAILED(Value.Device->CreateShaderResourceView(InputBuffer, &SRVDesc, &SRV)))
		{ SafeRelease(InputBuffer); return false; }

		D3D11_BUFFER_DESC OutputDesc = {};
		OutputDesc.ByteWidth = VectorBytes;
		OutputDesc.Usage = D3D11_USAGE_DEFAULT;
		OutputDesc.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
		OutputDesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
		ID3D11Buffer* OutputBuffer = nullptr;
		if (FAILED(Value.Device->CreateBuffer(&OutputDesc, nullptr, &OutputBuffer)))
		{ SafeRelease(SRV); SafeRelease(InputBuffer); return false; }
		D3D11_UNORDERED_ACCESS_VIEW_DESC UAVDesc = {};
		UAVDesc.Format = DXGI_FORMAT_R32_TYPELESS;
		UAVDesc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
		UAVDesc.Buffer.NumElements = VectorBytes / 4;
		UAVDesc.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
		ID3D11UnorderedAccessView* UAV = nullptr;
		if (FAILED(Value.Device->CreateUnorderedAccessView(OutputBuffer, &UAVDesc, &UAV)))
		{ SafeRelease(OutputBuffer); SafeRelease(SRV); SafeRelease(InputBuffer); return false; }

		D3D11_BUFFER_DESC ConstantDesc = {};
		ConstantDesc.ByteWidth = 16;
		ConstantDesc.Usage = D3D11_USAGE_IMMUTABLE;
		ConstantDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		const uint32_t Constants[4] = { (uint32_t)Count, 0, 0, 0 };
		D3D11_SUBRESOURCE_DATA ConstantData = { Constants, 0, 0 };
		ID3D11Buffer* ConstantBuffer = nullptr;
		if (FAILED(Value.Device->CreateBuffer(&ConstantDesc, &ConstantData, &ConstantBuffer)))
		{ SafeRelease(UAV); SafeRelease(OutputBuffer); SafeRelease(SRV); SafeRelease(InputBuffer); return false; }

		Value.DeviceContext->CSSetShader(Value.Shader, nullptr, 0);
		Value.DeviceContext->CSSetShaderResources(0, 1, &SRV);
		Value.DeviceContext->CSSetUnorderedAccessViews(0, 1, &UAV, nullptr);
		Value.DeviceContext->CSSetConstantBuffers(0, 1, &ConstantBuffer);
		Value.DeviceContext->Dispatch(((UINT)Count + 255) / 256, 1, 1);
		ID3D11ShaderResourceView* NullSRV = nullptr;
		ID3D11UnorderedAccessView* NullUAV = nullptr;
		Value.DeviceContext->CSSetShaderResources(0, 1, &NullSRV);
		Value.DeviceContext->CSSetUnorderedAccessViews(0, 1, &NullUAV, nullptr);

		D3D11_BUFFER_DESC StagingDesc = OutputDesc;
		StagingDesc.Usage = D3D11_USAGE_STAGING;
		StagingDesc.BindFlags = 0;
		StagingDesc.MiscFlags = 0;
		StagingDesc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
		ID3D11Buffer* Staging = nullptr;
		bool Success = SUCCEEDED(Value.Device->CreateBuffer(&StagingDesc, nullptr, &Staging));
		if (Success)
		{
			Value.DeviceContext->CopyResource(Staging, OutputBuffer);
			D3D11_MAPPED_SUBRESOURCE Mapped = {};
			Success = SUCCEEDED(Value.DeviceContext->Map(Staging, 0, D3D11_MAP_READ, 0, &Mapped));
			if (Success) { std::memcpy(Output, Mapped.pData, VectorBytes); Value.DeviceContext->Unmap(Staging, 0); }
		}
		SafeRelease(Staging); SafeRelease(ConstantBuffer); SafeRelease(UAV);
		SafeRelease(OutputBuffer); SafeRelease(SRV); SafeRelease(InputBuffer);
		return Success;
	}
#else
	bool IsFluidComputeShaderBackendAvailable() { return false; }
	bool ApplyFluidVelocityDeltaComputeShader(const Vector3*, const Vector3*, size_t, Vector3*) { return false; }
#endif
}

#endif
