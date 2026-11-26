#include "../Compute/BackendAvailability.h"
#include "FluidBackend.h"
#include "../Compute/MetalCompute.h"
#include <mutex>
#include <stdexcept>

namespace Riemann
{
    namespace
    {
        using namespace MetalCompute;
        const char *VelocityKernel = R"SHADER(
#include <metal_stdlib>
using namespace metal;


float ReadFloat(device uint* b, uint i) { return as_type<float>(b[i]); }

uint ReadUInt(device uint* b, uint i) { return b[i]; }

void WriteFloat(device uint* b, uint i, float v) { b[i] = as_type<uint>(v); }

void WriteUInt(device uint* b, uint i, uint v) { b[i] = v; }

uint Exchange(device uint* b, uint i, uint v) { return atomic_exchange_explicit((device atomic_uint*)(b + i), v, memory_order_relaxed); }

float PF(constant uint4* data, uint i) { return as_type<float>(data[i / 4][i % 4]); }

int PI(constant uint4* data, uint i) { return (int)data[i / 4][i % 4]; }


kernel void ApplyVelocityDelta(device uint* b0 [[buffer(0)]], device uint* b1 [[buffer(1)]], device uint* b2 [[buffer(2)]], device uint* b3 [[buffer(3)]], device uint* b4 [[buffer(4)]], device uint* b5 [[buffer(5)]], device uint* b6 [[buffer(6)]], device uint* b7 [[buffer(7)]], constant uint4* constants [[buffer(8)]], threadgroup float* shared0 [[threadgroup(0)]], uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]], uint3 gid [[threadgroup_position_in_grid]])

{

    uint I = tid.x;

    if (I >= (uint)PI(constants, 0)) return;

    WriteFloat(b2, I, ReadFloat(b0, I) + ReadFloat(b1, I));

}
)SHADER";
        struct Context
        {
            std::unique_ptr<MetalDevice> Compute;
            SharedProgram Kernel;
            SharedBuffer Velocity, Delta, Output;
            std::mutex Mutex;
            bool Attempted = false;
            bool Initialize()
            {
                if (Attempted)
                    return Kernel != nullptr;
                Attempted = true;
                try
                {
                    Compute = CreateMetalDevice();
                    if (!Compute)
                        return false;
                    Kernel = Compute->CreateProgram(VelocityKernel, "ApplyVelocityDelta");
                    return true;
                }
                catch (const std::exception &)
                {
                    return false;
                }
            }
        };
        Context &GetContext()
        {
            static Context Value;
            return Value;
        }
    } // namespace

    bool ApplyFluidVelocityDeltaMetal(const Vector3 *, const Vector3 *, size_t, Vector3 *);

    bool IsMetalComputeAvailable()
    {
        return MetalCompute::QueryBackendInfo().Available;
    }

    bool IsFluidMetalBackendAvailable()
    {
        return IsMetalComputeAvailable();
    }

    bool ApplyFluidVelocityDeltaMetal(const Vector3 *Velocity, const Vector3 *Delta, size_t Count, Vector3 *Output)
    {
        if (!Count)
            return true;
        if (!Velocity || !Delta || !Output || Count > (65535u * 256u) / 3u)
            return false;
        Context &Value = GetContext();
        std::lock_guard<std::mutex> Lock(Value.Mutex);
        if (!Value.Initialize())
            return false;
        try
        {
            const size_t Bytes = Count * sizeof(Vector3);
            ReserveMetalBuffer(*Value.Compute, Value.Velocity, Bytes);
            ReserveMetalBuffer(*Value.Compute, Value.Delta, Bytes);
            ReserveMetalBuffer(*Value.Compute, Value.Output, Bytes);
            Value.Compute->Upload(Value.Velocity, Velocity, Bytes);
            Value.Compute->Upload(Value.Delta, Delta, Bytes);
            Constants P;
            P.Integer(0, static_cast<int>(Count * 3));
            Value.Compute->Dispatch(Value.Kernel, {Value.Velocity, Value.Delta, Value.Output}, P,
                                    static_cast<int>(Count * 3));
            Value.Compute->Download({{Value.Output, Output, Bytes}});
            return true;
        }
        catch (const std::exception &)
        {
            Value.Compute->Finish();
            return false;
        }
    }
} // namespace Riemann
