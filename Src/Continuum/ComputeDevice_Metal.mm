#include "ComputeDevice.h"
#include "ComputeShaderSource.h"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#include <map>
#include <stdexcept>

namespace Riemann
{
namespace SnowCompute
{
	namespace
	{
		struct NativeBuffer final : Buffer { id<MTLBuffer> Resource; };
		struct NativeProgram final : Program { id<MTLComputePipelineState> Pipeline; };

		std::string ErrorText(NSError* error)
		{
			return error ? std::string(error.localizedDescription.UTF8String) : "Metal compute operation failed";
		}
		class MetalDevice final : public Device
		{
			id<MTLDevice> Handle;
			id<MTLCommandQueue> Queue;
			id<MTLCommandBuffer> Commands;
			id<MTLComputeCommandEncoder> Encoder;
			std::map<std::string, id<MTLLibrary>> Libraries;
			BackendInfo Info;

			void Submit(bool check)
			{
				if (!Commands) return;
				[Encoder endEncoding]; Encoder = nil;
				[Commands commit]; [Commands waitUntilCompleted];
				NSError* error = Commands.error;
				Commands = nil;
				if (check && error) throw std::runtime_error(ErrorText(error));
			}
		public:
			MetalDevice()
			{
				@autoreleasepool
				{
					Handle = MTLCreateSystemDefaultDevice();
					if (!Handle) throw std::runtime_error("No Metal compute device available");
					Queue = [Handle newCommandQueue];
					if (!Queue) throw std::runtime_error("Metal command queue creation failed");
					Info.Available = true; Info.API = "Metal"; Info.DeviceName = Handle.name.UTF8String;
					Info.Message = "Metal compute available";
				}
			}
			~MetalDevice() override { @autoreleasepool { Submit(false); } }
			SharedBuffer CreateBuffer(std::size_t bytes) override
			{
				@autoreleasepool
				{
					if (!bytes || bytes > 128u * 1024u * 1024u || bytes % 4)
						throw std::runtime_error("Compute buffer exceeds supported capacity");
					auto buffer = std::make_shared<NativeBuffer>(); buffer->Size = bytes;
					buffer->Resource = [Handle newBufferWithLength:bytes options:MTLResourceStorageModeShared];
					if (!buffer->Resource) throw std::runtime_error("Metal compute buffer allocation failed");
					return buffer;
				}
			}
			SharedProgram CreateProgram(const char* source, const char* entry) override
			{
				@autoreleasepool
				{
					auto& library = Libraries[source];
					if (!library)
					{
						MTLCompileOptions* options = [[MTLCompileOptions alloc] init];
						options.fastMathEnabled = NO;
						const std::string fullSource = std::string(MetalPrelude) + source;
						NSError* error = nil;
						library = [Handle newLibraryWithSource:[NSString stringWithUTF8String:fullSource.c_str()]
							options:options error:&error];
						if (!library) throw std::runtime_error(ErrorText(error));
					}
					id<MTLFunction> function = [library newFunctionWithName:[NSString stringWithUTF8String:entry]];
					if (!function) throw std::runtime_error(std::string("Metal kernel missing: ") + entry);
					auto program = std::make_shared<NativeProgram>(); NSError* error = nil;
					program->Pipeline = [Handle newComputePipelineStateWithFunction:function error:&error];
					if (!program->Pipeline) throw std::runtime_error(ErrorText(error));
					if (program->Pipeline.maxTotalThreadsPerThreadgroup < 256)
						throw std::runtime_error("Metal device cannot run 256-thread compute groups");
					return program;
				}
			}
			void Upload(const SharedBuffer& buffer, const void* data, std::size_t bytes) override
			{
				@autoreleasepool
				{
					if (!buffer || bytes > buffer->Size) throw std::runtime_error("Compute upload exceeds buffer size");
					Submit(true);
					std::memcpy(static_cast<NativeBuffer&>(*buffer).Resource.contents, data, bytes);
				}
			}
			void Dispatch(const SharedProgram& program, const Bindings& buffers,
						  const Constants& constants, int threads) override
			{
				@autoreleasepool
				{
					if (threads <= 0 || threads > 65535 * 256) throw std::runtime_error("Compute dispatch exceeds capacity");
					if (!Commands)
					{
						Commands = [Queue commandBuffer]; Encoder = [Commands computeCommandEncoder];
						if (!Encoder) throw std::runtime_error("Metal compute encoder creation failed");
					}
					else [Encoder memoryBarrierWithScope:MTLBarrierScopeBuffers];
					[Encoder setComputePipelineState:static_cast<NativeProgram&>(*program).Pipeline];
					for (std::size_t i = 0; i < buffers.size(); ++i)
						[Encoder setBuffer:buffers[i] ? static_cast<NativeBuffer&>(*buffers[i]).Resource : nil offset:0 atIndex:i];
					[Encoder setBytes:&constants length:sizeof(constants) atIndex:8];
					[Encoder setThreadgroupMemoryLength:256 * sizeof(float) atIndex:0];
					[Encoder dispatchThreadgroups:MTLSizeMake((threads + 255) / 256, 1, 1)
						threadsPerThreadgroup:MTLSizeMake(256, 1, 1)];
				}
			}
			void Download(const std::vector<Readback>& requests) override
			{
				@autoreleasepool
				{
					Submit(true);
					for (const auto& request : requests)
					{
						if (!request.Source || request.Bytes > request.Source->Size)
							throw std::runtime_error("Compute download exceeds buffer size");
						std::memcpy(request.Destination, static_cast<NativeBuffer&>(*request.Source).Resource.contents, request.Bytes);
					}
				}
			}
			void Finish() override { @autoreleasepool { Submit(false); } }
			BackendInfo GetInfo() const override { return Info; }
		};
	}
	std::unique_ptr<Device> CreateDevice() { return std::make_unique<MetalDevice>(); }
}
}
