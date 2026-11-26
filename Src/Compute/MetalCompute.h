#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <stdexcept>
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

namespace Riemann
{
    namespace MetalCompute
    {
        struct Buffer
        {
            std::size_t Size = 0;
            id<MTLBuffer> Resource;
        };
        struct Program
        {
            id<MTLComputePipelineState> Pipeline;
        };

        inline std::string ErrorText(NSError *error)
        {
            return error ? std::string(error.localizedDescription.UTF8String) : "Metal compute operation failed";
        }
        using SharedBuffer = std::shared_ptr<Buffer>;
        using SharedProgram = std::shared_ptr<Program>;
        using Bindings = std::array<SharedBuffer, 8>;
        struct Constants
        {
            std::array<std::uint32_t, 64> Words{};
            void Integer(std::size_t I, int V)
            {
                Words[I] = static_cast<std::uint32_t>(V);
            }
            void Float(std::size_t I, float V)
            {
                std::memcpy(&Words[I], &V, sizeof(V));
            }
        };
        struct Readback
        {
            SharedBuffer Source;
            void *Destination;
            std::size_t Bytes;
        };
        struct BackendInfo
        {
            bool Available = false;
            std::string API, DeviceName, Message;
        };
        inline const BackendInfo& QueryBackendInfo()
        {
            static const BackendInfo Info = []
            {
                @autoreleasepool
                {
                    BackendInfo Result;
                    Result.API = "Metal";
                    id<MTLDevice> Device = MTLCreateSystemDefaultDevice();
                    Result.Available = Device != nil;
                    if (Device) Result.DeviceName = Device.name.UTF8String;
                    Result.Message = Device ? "Metal built; device detected" : "No Metal device detected";
                    return Result;
                }
            }();
            return Info;
        }
        class MetalDevice final
        {
            id<MTLDevice> Handle;
            id<MTLCommandQueue> Queue;
            id<MTLCommandBuffer> Commands;
            id<MTLComputeCommandEncoder> Encoder;
            std::map<std::string, id<MTLLibrary>> Libraries;
            BackendInfo Info;

            void Submit(bool check)
            {
                if (!Commands)
                    return;
                [Encoder endEncoding];
                Encoder = nil;
                [Commands commit];
                [Commands waitUntilCompleted];
                NSError *error = Commands.error;
                Commands = nil;
                if (check && error)
                    throw std::runtime_error(ErrorText(error));
            }

          public:
            MetalDevice()
            {
                @autoreleasepool
                {
                    Handle = MTLCreateSystemDefaultDevice();
                    if (!Handle)
                        throw std::runtime_error("No Metal compute device available");
                    Queue = [Handle newCommandQueue];
                    if (!Queue)
                        throw std::runtime_error("Metal command queue creation failed");
                    Info.Available = true;
                    Info.API = "Metal";
                    Info.DeviceName = Handle.name.UTF8String;
                    Info.Message = "Metal compute available";
                }
            }
            ~MetalDevice()
            {
                @autoreleasepool
                {
                    Submit(false);
                }
            }
            SharedBuffer CreateBuffer(std::size_t bytes)
            {
                @autoreleasepool
                {
                    if (!bytes || bytes > 128u * 1024u * 1024u || bytes % 4)
                        throw std::runtime_error("Compute buffer exceeds supported capacity");
                    auto buffer = std::make_shared<Buffer>();
                    buffer->Size = bytes;
                    buffer->Resource = [Handle newBufferWithLength:bytes options:MTLResourceStorageModeShared];
                    if (!buffer->Resource)
                        throw std::runtime_error("Metal compute buffer allocation failed");
                    return buffer;
                }
            }
            SharedProgram CreateProgram(const char *source, const char *entry)
            {
                @autoreleasepool
                {
                    auto &library = Libraries[source];
                    if (!library)
                    {
                        MTLCompileOptions *options = [[MTLCompileOptions alloc] init];
                        options.fastMathEnabled = NO;
                        const std::string fullSource = std::string(source);
                        NSError *error = nil;
                        library = [Handle newLibraryWithSource:[NSString stringWithUTF8String:fullSource.c_str()]
                                                       options:options
                                                         error:&error];
                        if (!library)
                            throw std::runtime_error(ErrorText(error));
                    }
                    id<MTLFunction> function = [library newFunctionWithName:[NSString stringWithUTF8String:entry]];
                    if (!function)
                        throw std::runtime_error(std::string("Metal kernel missing: ") + entry);
                    auto program = std::make_shared<Program>();
                    NSError *error = nil;
                    program->Pipeline = [Handle newComputePipelineStateWithFunction:function error:&error];
                    if (!program->Pipeline)
                        throw std::runtime_error(ErrorText(error));
                    if (program->Pipeline.maxTotalThreadsPerThreadgroup < 256)
                        throw std::runtime_error("Metal device cannot run 256-thread compute groups");
                    return program;
                }
            }
            void Upload(const SharedBuffer &buffer, const void *data, std::size_t bytes)
            {
                @autoreleasepool
                {
                    if (!buffer || bytes > buffer->Size)
                        throw std::runtime_error("Compute upload exceeds buffer size");
                    Submit(true);
                    std::memcpy(static_cast<Buffer &>(*buffer).Resource.contents, data, bytes);
                }
            }
            void Dispatch(const SharedProgram &program, const Bindings &buffers, const Constants &constants,
                          int threads)
            {
                @autoreleasepool
                {
                    if (threads <= 0 || threads > 65535 * 256)
                        throw std::runtime_error("Compute dispatch exceeds capacity");
                    if (!Commands)
                    {
                        Commands = [Queue commandBuffer];
                        Encoder = [Commands computeCommandEncoder];
                        if (!Encoder)
                            throw std::runtime_error("Metal compute encoder creation failed");
                    }
                    else
                        [Encoder memoryBarrierWithScope:MTLBarrierScopeBuffers];
                    [Encoder setComputePipelineState:static_cast<Program &>(*program).Pipeline];
                    for (std::size_t i = 0; i < buffers.size(); ++i)
                        [Encoder setBuffer:buffers[i] ? static_cast<Buffer &>(*buffers[i]).Resource : nil
                                    offset:0
                                   atIndex:i];
                    [Encoder setBytes:&constants length:sizeof(constants) atIndex:8];
                    [Encoder setThreadgroupMemoryLength:256 * sizeof(float) atIndex:0];
                    [Encoder dispatchThreadgroups:MTLSizeMake((threads + 255) / 256, 1, 1)
                            threadsPerThreadgroup:MTLSizeMake(256, 1, 1)];
                }
            }
            void Download(const std::vector<Readback> &requests)
            {
                @autoreleasepool
                {
                    Submit(true);
                    for (const auto &request : requests)
                    {
                        if (!request.Source || request.Bytes > request.Source->Size)
                            throw std::runtime_error("Compute download exceeds buffer size");
                        std::memcpy(request.Destination, static_cast<Buffer &>(*request.Source).Resource.contents,
                                    request.Bytes);
                    }
                }
            }
            void Finish()
            {
                @autoreleasepool
                {
                    Submit(false);
                }
            }
            BackendInfo GetInfo() const
            {
                return Info;
            }
        };
        inline std::unique_ptr<MetalDevice> CreateMetalDevice()
        {
            return std::make_unique<MetalDevice>();
        }
        inline void ReserveMetalBuffer(MetalDevice &Compute, SharedBuffer &Resource, std::size_t Bytes)
        {
            if (!Resource || Resource->Size < Bytes)
                Resource = Compute.CreateBuffer(Bytes);
        }
    } // namespace MetalCompute
} // namespace Riemann
