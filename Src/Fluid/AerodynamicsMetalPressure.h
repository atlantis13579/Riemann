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

int Index(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
          constant uint4 *constants, int3 P)
{
    return P.x + int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).x *
                     (P.y + int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).y * P.z);
}

int3 Position(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
              constant uint4 *constants, int I)
{
    return int3(I % int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).x,
                I / int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).x %
                    int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).y,
                I / (int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).x *
                     int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).y));
}

bool Inside(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
            constant uint4 *constants, int3 P)
{
    return all(P >= 0) && all(P < int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)));
}

bool Solid(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
           constant uint4 *constants, int I)
{
    return ReadFloat(b2, I * 4 + 3) != 0;
}

bool Closed(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
            constant uint4 *constants, int3 P)

{

    return P.x < 0 || P.x >= int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).x || P.y < 0 || P.z < 0 ||
           P.z >= int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).z ||
           (Inside(b0, b1, b2, b3, b4, b5, constants, P) &&
            Solid(b0, b1, b2, b3, b4, b5, constants, Index(b0, b1, b2, b3, b4, b5, constants, P)));
}

kernel void InitializeState(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                            device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                            device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                            constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                            uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                            uint3 gid [[threadgroup_position_in_grid]])

{

    int I = int(tid.x);
    if (I >= (int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).x *
              int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).y *
              int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).z))
        return;
    int3 P = Position(b0, b1, b2, b3, b4, b5, constants, I);
    float Base = 0, Div = 0;

    for (int A = 0; A < 3; ++A)

    {

        int3 N = int3(0, 0, 0);
        N[A] = 1;

        Base += float3(PF(constants, 4), PF(constants, 5), PF(constants, 6))[A] *
                ((Closed(b0, b1, b2, b3, b4, b5, constants, P - N) ? 0 : 1) +
                 (Closed(b0, b1, b2, b3, b4, b5, constants, P + N) ? 0 : 1));

        float Low = Inside(b0, b1, b2, b3, b4, b5, constants, P - N)
                        ? ReadFloat(b1, PI(constants, 23) + Index(b0, b1, b2, b3, b4, b5, constants, P - N) * 4 + A)
                        : 0;

        Div += (ReadFloat(b1, PI(constants, 23) + I * 4 + A) - Low) *
               sqrt(float3(PF(constants, 4), PF(constants, 5), PF(constants, 6))[A]);
    }

    float D = Base;

    int Begin = int(ReadUInt(b3, PI(constants, 14) + I * 2)), End = int(ReadUInt(b3, PI(constants, 14) + I * 2 + 1));

    for (int J = Begin; J < End; ++J)

    {

        int Row = int(ReadUInt(b3, PI(constants, 15) + J * 2));
        float Sum = 0;

        for (int K = int(ReadUInt(b3, PI(constants, 12) + Row)); K < int(ReadUInt(b3, PI(constants, 12) + Row + 1));
             ++K)

            Sum += abs(ReadFloat(b3, PI(constants, 13) + K * 2 + 1));

        D += abs(ReadFloat(b3, PI(constants, 15) + J * 2 + 1)) * Sum;
    }

    bool Active = !Solid(b0, b1, b2, b3, b4, b5, constants, I) && D > 0;

    WriteFloat(b0, I * 3, Active ? -Div : 0);
    WriteFloat(b0, I * 3 + 1, Active ? 1 / D : 0);
    WriteFloat(b0, I * 3 + 2, Base);

    WriteFloat(b0, PI(constants, 20) + I, 0);
    WriteFloat(b0, PI(constants, 21) + I, 0);
}

kernel void SurfaceResponse(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                            device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                            device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                            constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                            uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                            uint3 gid [[threadgroup_position_in_grid]])

{

    int I = int(tid.x);
    if (I >= PI(constants, 3))
        return;
    float V = 0;

    for (int J = int(ReadUInt(b3, PI(constants, 12) + I)); J < int(ReadUInt(b3, PI(constants, 12) + I + 1)); ++J)

        V += ReadFloat(b3, PI(constants, 13) + J * 2 + 1) *
             ReadFloat(b0, PI(constants, 20) + ReadUInt(b3, PI(constants, 13) + J * 2));

    WriteFloat(b4, I, V);
}

kernel void RelaxPressure(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                          device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                          device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                          constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                          uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                          uint3 gid [[threadgroup_position_in_grid]])

{

    int I = int(tid.x);
    if (I >= (int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).x *
              int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).y *
              int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).z))
        return;
    int3 P = Position(b0, b1, b2, b3, b4, b5, constants, I);

    float X = ReadFloat(b0, PI(constants, 20) + I), V = ReadFloat(b0, I * 3 + 2) * X;

    for (int A = 0; A < 3; ++A)
        for (int Sign = -1; Sign <= 1; Sign += 2)

        {

            int3 Q = P;
            Q[A] += Sign;

            if (Inside(b0, b1, b2, b3, b4, b5, constants, Q) && !Closed(b0, b1, b2, b3, b4, b5, constants, Q))
                V -= float3(PF(constants, 4), PF(constants, 5), PF(constants, 6))[A] *
                     ReadFloat(b0, PI(constants, 20) + Index(b0, b1, b2, b3, b4, b5, constants, Q));
        }

    for (int J = int(ReadUInt(b3, PI(constants, 14) + I * 2)); J < int(ReadUInt(b3, PI(constants, 14) + I * 2 + 1));
         ++J)

        V += ReadFloat(b3, PI(constants, 15) + J * 2 + 1) * ReadFloat(b4, ReadUInt(b3, PI(constants, 15) + J * 2));

    float Step = PF(constants, 8) * ReadFloat(b0, I * 3 + 1) * (ReadFloat(b0, I * 3) - V)

                 + PF(constants, 9) * (X - ReadFloat(b0, PI(constants, 21) + I));

    WriteFloat(b0, PI(constants, 22) + I, X + Step);
}

kernel void ApplyCorrection(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                            device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                            device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                            constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                            uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                            uint3 gid [[threadgroup_position_in_grid]])

{

    int I = int(tid.x);
    if (I >= (int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).x *
              int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).y *
              int3(PI(constants, 0), PI(constants, 1), PI(constants, 2)).z))
        return;
    int3 P = Position(b0, b1, b2, b3, b4, b5, constants, I);

    for (int A = 0; A < 3; ++A)

    {

        float W = ReadFloat(b2, I * 4 + A), V = ReadFloat(b1, PI(constants, 23) + I * 4 + A);

        uint F = ReadUInt(b3, PI(constants, 17) + I * 3 + A);

        if (F != 0xffffffffu)

            for (int N = 0; N < 3; ++N)
                W += ReadFloat(b3, PI(constants, 16) + F * 8 + 5 + N) *
                     ReadFloat(b4, ReadUInt(b3, PI(constants, 16) + F * 8 + 2 + N));

        int3 Q = P;
        Q[A] += 1;

        if (Solid(b0, b1, b2, b3, b4, b5, constants, I) || Closed(b0, b1, b2, b3, b4, b5, constants, Q))
            V = W;

        else
            V -= ((Inside(b0, b1, b2, b3, b4, b5, constants, Q)
                       ? ReadFloat(b0, PI(constants, 20) + Index(b0, b1, b2, b3, b4, b5, constants, Q))
                       : 0) -
                  ReadFloat(b0, PI(constants, 20) + I)) *
                 sqrt(float3(PF(constants, 4), PF(constants, 5), PF(constants, 6))[A]);

        WriteFloat(b2, I * 4 + A, W);
        WriteFloat(b1, PI(constants, 23) + I * 4 + A, V);
    }

    WriteFloat(b5, I, ReadFloat(b0, PI(constants, 20) + I));
}
)SHADER"
