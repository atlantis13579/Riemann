// Native Metal kernels.
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
float2 GetBounds(device uint *b0, device uint *b1, constant uint4 *constants, uint VectorOffset, uint Row)
{
    float LowerBound = ReadFloat(b0, (((uint)PI(constants, 2))) / 4 + (VectorOffset + Row));
    float UpperBound = ReadFloat(b0, (((uint)PI(constants, 3))) / 4 + (VectorOffset + Row));
    int NormalRow = ((int)ReadUInt(b0, (((uint)PI(constants, 6))) / 4 + (VectorOffset + Row)));
    if (NormalRow >= 0)
    {
        float Limit = ReadFloat(b0, (((uint)PI(constants, 4))) / 4 + (VectorOffset + Row)) *
                      max(0.0f, ReadFloat(b1, ((uint)PI(constants, 10)) / 4 + (VectorOffset + (uint)NormalRow)));
        LowerBound = max(LowerBound, -Limit);
        UpperBound = min(UpperBound, Limit);
    }
    return float2(LowerBound, UpperBound);
}
kernel void SolveProjectedGaussSeidel(
    device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
    device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
    device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]], constant uint4 *constants [[buffer(8)]],
    threadgroup float *shared0 [[threadgroup(0)]], uint3 tid [[thread_position_in_grid]],
    uint3 lid [[thread_position_in_threadgroup]], uint3 gid [[threadgroup_position_in_grid]])
{
    uint3 GroupId = gid, GroupThreadId = lid;
    float SharedResidual = 0;
    uint SharedIterations = 0, SharedConverged = 0;
    const uint LocalIsland = GroupId.x;
    if (LocalIsland >= ((uint)PI(constants, 13)))
    {
        return;
    }
    const uint Thread = GroupThreadId.x;
    const uint Island = ((uint)PI(constants, 12)) + LocalIsland;
    const uint Dimension = (uint)((int)ReadUInt(b0, (((uint)PI(constants, 9))) / 4 + (Island)));
    const uint MatrixOffset = (uint)((int)ReadUInt(b0, (((uint)PI(constants, 7))) / 4 + (Island)));
    const uint VectorOffset = (uint)((int)ReadUInt(b0, (((uint)PI(constants, 8))) / 4 + (Island)));
    for (uint InitialRow = Thread; InitialRow < Dimension; InitialRow += 256)
    {
        WriteFloat(b1, ((uint)PI(constants, 10)) / 4 + (VectorOffset + InitialRow),
                   ReadFloat(b0, (((uint)PI(constants, 5))) / 4 + (VectorOffset + InitialRow)));
    }
    threadgroup_barrier(mem_flags::mem_device | mem_flags::mem_threadgroup);
    if (Thread == 0)
    {
        SharedIterations = 0;
        SharedConverged = 0;
        SharedResidual = 0.0f;
        for (uint ClampRow = 0; ClampRow < Dimension; ++ClampRow)
        {
            float2 Bounds = GetBounds(b0, b1, constants, VectorOffset, ClampRow);
            float LowerBound = Bounds.x, UpperBound = Bounds.y;
            WriteFloat(b1, ((uint)PI(constants, 10)) / 4 + (VectorOffset + ClampRow),
                       max(LowerBound,
                           min(UpperBound, ReadFloat(b1, ((uint)PI(constants, 10)) / 4 + (VectorOffset + ClampRow)))));
        }
    }
    threadgroup_barrier(mem_flags::mem_device | mem_flags::mem_threadgroup);
    for (uint Iteration = 0; Iteration < ((uint)PI(constants, 14)); ++Iteration)
    {
        for (uint SolveRow = 0; SolveRow < Dimension; ++SolveRow)
        {
            float Partial = 0.0f;
            for (uint Column = Thread; Column < Dimension; Column += 256)
            {
                Partial +=
                    ReadFloat(b0, (((uint)PI(constants, 0))) / 4 + (MatrixOffset + SolveRow * Dimension + Column)) *
                    ReadFloat(b1, ((uint)PI(constants, 10)) / 4 + (VectorOffset + Column));
            }
            shared0[Thread] = Partial;
            threadgroup_barrier(mem_flags::mem_threadgroup);
            for (uint Stride = 128; Stride > 0; Stride >>= 1)
            {
                if (Thread < Stride)
                {
                    shared0[Thread] += shared0[Thread + Stride];
                }
                threadgroup_barrier(mem_flags::mem_threadgroup);
            }
            if (Thread == 0)
            {
                float2 Bounds = GetBounds(b0, b1, constants, VectorOffset, SolveRow);
                float LowerBound = Bounds.x, UpperBound = Bounds.y;
                const float OldLambda = ReadFloat(b1, ((uint)PI(constants, 10)) / 4 + (VectorOffset + SolveRow));
                const float Diagonal =
                    ReadFloat(b0, (((uint)PI(constants, 0))) / 4 + (MatrixOffset + SolveRow * Dimension + SolveRow));
                const float W = ReadFloat(b0, (((uint)PI(constants, 1))) / 4 + (VectorOffset + SolveRow)) + shared0[0];
                const float NewLambda = max(LowerBound, min(UpperBound, OldLambda - PF(constants, 16) * W / Diagonal));
                WriteFloat(b1, ((uint)PI(constants, 10)) / 4 + (VectorOffset + SolveRow), NewLambda);
            }
            threadgroup_barrier(mem_flags::mem_device | mem_flags::mem_threadgroup);
        }
        if (Thread == 0)
        {
            SharedIterations = Iteration + 1;
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
    }
    if (Thread == 0)
    {
        SharedResidual = 0.0f;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);
    for (uint ResidualRow = 0; ResidualRow < Dimension; ++ResidualRow)
    {
        float Partial = 0.0f;
        for (uint Column = Thread; Column < Dimension; Column += 256)
        {
            Partial +=
                ReadFloat(b0, (((uint)PI(constants, 0))) / 4 + (MatrixOffset + ResidualRow * Dimension + Column)) *
                ReadFloat(b1, ((uint)PI(constants, 10)) / 4 + (VectorOffset + Column));
        }
        shared0[Thread] = Partial;
        threadgroup_barrier(mem_flags::mem_threadgroup);
        for (uint Stride = 128; Stride > 0; Stride >>= 1)
        {
            if (Thread < Stride)
            {
                shared0[Thread] += shared0[Thread + Stride];
            }
            threadgroup_barrier(mem_flags::mem_threadgroup);
        }
        if (Thread == 0)
        {
            float2 Bounds = GetBounds(b0, b1, constants, VectorOffset, ResidualRow);
            float LowerBound = Bounds.x, UpperBound = Bounds.y;
            const float Lambda = ReadFloat(b1, ((uint)PI(constants, 10)) / 4 + (VectorOffset + ResidualRow));
            const float Diagonal =
                ReadFloat(b0, (((uint)PI(constants, 0))) / 4 + (MatrixOffset + ResidualRow * Dimension + ResidualRow));
            const float W = ReadFloat(b0, (((uint)PI(constants, 1))) / 4 + (VectorOffset + ResidualRow)) + shared0[0];
            const float Projected = max(LowerBound, min(UpperBound, Lambda - W / Diagonal));
            SharedResidual = max(SharedResidual, abs(Projected - Lambda));
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
    }
    if (Thread == 0)
    {
        SharedConverged = SharedResidual <= PF(constants, 15) ? 1 : 0;
        const uint ResultAddress = ((uint)PI(constants, 11)) + Island * 16;
        WriteUInt(b1, (ResultAddress + 0) / 4, SharedIterations);
        WriteFloat(b1, (ResultAddress + 4) / 4, SharedResidual);
        WriteUInt(b1, (ResultAddress + 8) / 4, SharedConverged);
        WriteUInt(b1, (ResultAddress + 12) / 4, 0);
    }
}
)SHADER"
