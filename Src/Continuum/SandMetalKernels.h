// Native Metal sand kernels.
R"SHADER(

#include <metal_stdlib>
using namespace metal;

float ReadFloat(device uint *b, uint i)
{
    return as_type<float>(b[i]);
}

uint ReadUInt(device uint *b, uint i)
{
    return b[i];
}

void WriteFloat(device uint *b, uint i, float v)
{
    b[i] = as_type<uint>(v);
}

void WriteUInt(device uint *b, uint i, uint v)
{
    b[i] = v;
}

uint Exchange(device uint *b, uint i, uint v)
{
    return atomic_exchange_explicit((device atomic_uint *)(b + i), v, memory_order_relaxed);
}

float PF(constant uint4 *data, uint i)
{
    return as_type<float>(data[i / 4][i % 4]);
}

int PI(constant uint4 *data, uint i)
{
    return (int)data[i / 4][i % 4];
}

int SandX(int k)
{
    return k == 0 || k == 4 || k == 6 ? -1 : (k == 1 || k == 5 || k == 7 ? 1 : 0);
}

int SandZ(int k)
{
    return k == 2 || k == 4 || k == 7 ? -1 : (k == 3 || k == 5 || k == 6 ? 1 : 0);
}

kernel void SandFlux(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                     device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                     device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                     constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                     uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                     uint3 gid [[threadgroup_position_in_grid]])

{

    int i = (int)tid.x, n = PI(constants, 0);
    if (i >= n * n)
        return;

    int x = i % n, z = i / n;
    float h = ReadFloat(b0, i), sum = 0;
    float flow[8];

    for (int k = 0; k < 8; ++k)

    {

        int nx = x + SandX(k), nz = z + SandZ(k), j = nz * n + nx;
        float f = 0;

        if (nx >= 0 && nx < n && nz >= 0 && nz < n)

            if (ReadFloat(b0, j) < ReadFloat(b2, j) - 1.e-6)

                f = max(0.0, h - ReadFloat(b0, j) - PF(constants, 1) * (k < 4 ? 1.0 : 1.41421356237)) *
                    PF(constants, 2);

        flow[k] = f;
        sum += f;
    }

    float scale = sum > h ? h / sum : 1.0;

    for (int k = 0; k < 8; ++k)
        WriteFloat(b1, i * 8 + k, flow[k] * scale);
}

kernel void SandGather(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                       device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                       device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                       constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                       uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                       uint3 gid [[threadgroup_position_in_grid]])

{

    int i = (int)tid.x, n = PI(constants, 0);
    if (i >= n * n)
        return;

    int x = i % n, z = i / n;
    float h = ReadFloat(b0, i);

    for (int k = 0; k < 8; ++k)

    {

        h -= ReadFloat(b1, i * 8 + k);
        int nx = x + SandX(k), nz = z + SandZ(k);

        if (nx >= 0 && nx < n && nz >= 0 && nz < n)
            h += ReadFloat(b1, (nz * n + nx) * 8 + (k ^ 1));
    }

    WriteFloat(b3, i, max(0.0, h));
}
)SHADER"
