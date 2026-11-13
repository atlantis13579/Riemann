#include "ComputeDevice.h"

#if defined(_WIN32) && defined(RIEMANN_SNOW_COMPUTE)
#define NOMINMAX
#include "ComputeShaderSource.h"
#include <d3d11.h>
#include <d3d11sdklayers.h>
#include <d3dcompiler.h>
#include <dxgi.h>
#include <wrl/client.h>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>

namespace Riemann
{
namespace SnowCompute
{
	namespace
	{
		using Microsoft::WRL::ComPtr;
		void Check(HRESULT result, const char* operation)
		{
			if (FAILED(result)) throw std::runtime_error(std::string(operation) + " failed (HRESULT " +
				std::to_string(static_cast<unsigned long>(result)) + ")");
		}
		struct NativeBuffer final : Buffer
		{
			ComPtr<ID3D11Buffer> Resource, Staging;
			ComPtr<ID3D11UnorderedAccessView> View;
		};
		struct NativeProgram final : Program { ComPtr<ID3D11ComputeShader> Shader; };

		class DX11Device final : public Device
		{
			ComPtr<ID3D11Device> Handle;
			ComPtr<ID3D11DeviceContext> Context;
			ComPtr<ID3D11Buffer> ParameterBuffer;
			ComPtr<ID3D11InfoQueue> Validation;
			BackendInfo Info;
			void CheckValidation()
			{
				if (!Validation) return;
				const auto count = Validation->GetNumStoredMessagesAllowedByRetrievalFilter();
				for (UINT64 i = 0; i < count; ++i)
				{
					SIZE_T bytes = 0;
					Check(Validation->GetMessage(i, nullptr, &bytes), "Compute validation message size");
					std::vector<unsigned char> storage(bytes);
					auto* message = reinterpret_cast<D3D11_MESSAGE*>(storage.data());
					Check(Validation->GetMessage(i, message, &bytes), "Compute validation message");
					if (message->Severity <= D3D11_MESSAGE_SEVERITY_WARNING)
						throw std::runtime_error(std::string("DX11 validation: ") + message->pDescription);
				}
				Validation->ClearStoredMessages();
			}
		public:
			DX11Device()
			{
				const D3D_FEATURE_LEVEL level = D3D_FEATURE_LEVEL_11_0;
				UINT flags = 0;
#if defined(_DEBUG)
				flags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
				HRESULT result = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, flags,
					&level, 1, D3D11_SDK_VERSION, &Handle, nullptr, &Context);
				// Graphics Tools is optional even in a Debug build.
				if (result == DXGI_ERROR_SDK_COMPONENT_MISSING)
					result = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0,
						&level, 1, D3D11_SDK_VERSION, &Handle, nullptr, &Context);
				Check(result, "DX11 compute device creation");
				ComPtr<IDXGIDevice> dxgiDevice; ComPtr<IDXGIAdapter> adapter;
				Check(Handle.As(&dxgiDevice), "DXGI device query");
				Check(dxgiDevice->GetAdapter(&adapter), "DXGI adapter query");
				DXGI_ADAPTER_DESC description{};
				Check(adapter->GetDesc(&description), "DXGI adapter description");
				char name[512]{};
				WideCharToMultiByte(CP_UTF8, 0, description.Description, -1, name, sizeof(name), nullptr, nullptr);
				Info.Available = true; Info.API = "DX11"; Info.DeviceName = name;
				Info.Message = "Direct3D 11 Shader Model 5 compute available";
				if (SUCCEEDED(Handle.As(&Validation))) Info.Message += " (debug layer enabled)";
				D3D11_BUFFER_DESC desc{};
				desc.ByteWidth = sizeof(Constants); desc.Usage = D3D11_USAGE_DYNAMIC;
				desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER; desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
				Check(Handle->CreateBuffer(&desc, nullptr, &ParameterBuffer), "Compute constant buffer creation");
			}
			SharedBuffer CreateBuffer(std::size_t bytes) override
			{
				if (!bytes || bytes > 128u * 1024u * 1024u || bytes % 4)
					throw std::runtime_error("Compute buffer exceeds supported capacity");
				auto buffer = std::make_shared<NativeBuffer>(); buffer->Size = bytes;
				D3D11_BUFFER_DESC desc{};
				desc.ByteWidth = static_cast<UINT>(bytes); desc.Usage = D3D11_USAGE_DEFAULT;
				desc.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
				desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
				Check(Handle->CreateBuffer(&desc, nullptr, &buffer->Resource), "Compute buffer creation");
				D3D11_UNORDERED_ACCESS_VIEW_DESC view{};
				view.Format = DXGI_FORMAT_R32_TYPELESS; view.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
				view.Buffer.NumElements = desc.ByteWidth / 4; view.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
				Check(Handle->CreateUnorderedAccessView(buffer->Resource.Get(), &view, &buffer->View), "Compute UAV creation");
				return buffer;
			}
			SharedProgram CreateProgram(const char* source, const char* entry) override
			{
				// Bytecode is device independent; copying a simulation must not recompile all kernels.
				static std::mutex mutex;
				static std::map<std::string, ComPtr<ID3DBlob>> cache;
				std::lock_guard<std::mutex> lock(mutex);
				const std::string key = std::string(entry) + source;
				auto& bytecode = cache[key];
				if (!bytecode)
				{
					const std::string fullSource = std::string(HlslPrelude) + source;
					ComPtr<ID3DBlob> errors;
					const HRESULT result = D3DCompile(fullSource.data(), fullSource.size(), "SnowCompute", nullptr, nullptr,
						entry, "cs_5_0", D3DCOMPILE_OPTIMIZATION_LEVEL3 | D3DCOMPILE_IEEE_STRICTNESS, 0, &bytecode, &errors);
					if (FAILED(result)) throw std::runtime_error(std::string("Compute shader ") + entry + ": " +
						(errors ? static_cast<const char*>(errors->GetBufferPointer()) : "compilation failed"));
				}
				auto program = std::make_shared<NativeProgram>();
				Check(Handle->CreateComputeShader(bytecode->GetBufferPointer(), bytecode->GetBufferSize(), nullptr,
					&program->Shader), "Compute shader creation");
				return program;
			}
			void Upload(const SharedBuffer& buffer, const void* data, std::size_t bytes) override
			{
				if (!buffer || bytes > buffer->Size) throw std::runtime_error("Compute upload exceeds buffer size");
				D3D11_BOX box{0, 0, 0, static_cast<UINT>(bytes), 1, 1};
				Context->UpdateSubresource(static_cast<NativeBuffer&>(*buffer).Resource.Get(), 0, &box, data, 0, 0);
			}
			void Dispatch(const SharedProgram& program, const Bindings& buffers,
						  const Constants& constants, int threads) override
			{
				if (threads <= 0 || threads > 65535 * 256) throw std::runtime_error("Compute dispatch exceeds capacity");
				D3D11_MAPPED_SUBRESOURCE mapped{};
				Check(Context->Map(ParameterBuffer.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped), "Compute parameter upload");
				std::memcpy(mapped.pData, &constants, sizeof(constants)); Context->Unmap(ParameterBuffer.Get(), 0);
				ID3D11UnorderedAccessView* views[8]{};
				for (std::size_t i = 0; i < buffers.size(); ++i)
					if (buffers[i]) views[i] = static_cast<NativeBuffer&>(*buffers[i]).View.Get();
				Context->CSSetShader(static_cast<NativeProgram&>(*program).Shader.Get(), nullptr, 0);
				Context->CSSetConstantBuffers(0, 1, ParameterBuffer.GetAddressOf());
				Context->CSSetUnorderedAccessViews(0, 8, views, nullptr);
				Context->Dispatch(static_cast<UINT>((threads + 255) / 256), 1, 1);
			}
			void Download(const std::vector<Readback>& requests) override
			{
				ID3D11UnorderedAccessView* empty[8]{};
				Context->CSSetUnorderedAccessViews(0, 8, empty, nullptr);
				for (const auto& request : requests)
				{
					if (!request.Source || request.Bytes > request.Source->Size)
						throw std::runtime_error("Compute download exceeds buffer size");
					auto& buffer = static_cast<NativeBuffer&>(*request.Source);
					if (!buffer.Staging)
					{
						D3D11_BUFFER_DESC desc{};
						desc.ByteWidth = static_cast<UINT>(buffer.Size); desc.Usage = D3D11_USAGE_STAGING;
						desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
						Check(Handle->CreateBuffer(&desc, nullptr, &buffer.Staging), "Compute readback creation");
					}
					Context->CopyResource(buffer.Staging.Get(), buffer.Resource.Get());
				}
				for (const auto& request : requests)
				{
					auto& buffer = static_cast<NativeBuffer&>(*request.Source);
					D3D11_MAPPED_SUBRESOURCE mapped{};
					Check(Context->Map(buffer.Staging.Get(), 0, D3D11_MAP_READ, 0, &mapped), "Compute readback");
					std::memcpy(request.Destination, mapped.pData, request.Bytes);
					Context->Unmap(buffer.Staging.Get(), 0);
				}
				Check(Handle->GetDeviceRemovedReason(), "Compute device");
				CheckValidation();
			}
			void Finish() override { Context->ClearState(); Context->Flush(); }
			BackendInfo GetInfo() const override { return Info; }
		};
	}
	std::unique_ptr<Device> CreateDevice() { return std::make_unique<DX11Device>(); }
}
}
#endif
