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

float4 Read4(device uint *B, uint I)
{
    return float4(ReadFloat(B, I), ReadFloat(B, I + 1), ReadFloat(B, I + 2), ReadFloat(B, I + 3));
}

void Write4(device uint *B, uint I, float4 V)
{
    WriteFloat(B, I, V.x);
    WriteFloat(B, I + 1, V.y);
    WriteFloat(B, I + 2, V.z);
    WriteFloat(B, I + 3, V.w);
}

uint Index(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
           device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    return uint(P.x + int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                          (P.y + int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y * P.z));
}

float4 ReadVIn(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
               device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    return Read4(b0, PI(constants, 28) + 4 * Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P));
}

float2 ReadSIn(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
               device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    uint I = PI(constants, 30) + 2 * Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P);
    return float2(ReadFloat(b1, I), ReadFloat(b1, I + 1));
}

float2 ReadSForward(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                    device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    uint I = PI(constants, 33) + 2 * Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P);
    return float2(ReadFloat(b1, I), ReadFloat(b1, I + 1));
}

float4 ReadCurlIn(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
                  device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    return Read4(b2, 4 * Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P));
}

float ReadPressureIn(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                     device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    return ReadFloat(b3, Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P));
}

float ReadPressureOut(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                      device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    return ReadFloat(b3, Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P));
}

float ReadSolidIn(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
                  device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    return ReadFloat(b4, 4 * Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P) + 3);
}

float4 ReadWallIn(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
                  device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    return Read4(b4, 4 * Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P));
}

float ReadDivergenceIn(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                       device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    return ReadFloat(b5, Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P));
}

void WriteVOut(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
               device uint *b6, device uint *b7, constant uint4 *constants, int3 P, float4 V)
{
    if (PI(constants, 32) == 2)
        Write4(b2, 4 * Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P), V);
    else
        Write4(b0, PI(constants, 29) + 4 * Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P), V);
}

void WriteSOut(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
               device uint *b6, device uint *b7, constant uint4 *constants, int3 P, float2 V)
{
    uint I = PI(constants, 31) + 2 * Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P);
    WriteFloat(b1, I, V.x);
    WriteFloat(b1, I + 1, V.y);
}

void WritePressureOut(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                      device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, int3 P, float V)
{
    WriteFloat(b3, Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P), V);
}

void WriteDivergenceOut(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                        device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, int3 P, float V)
{
    WriteFloat(b5, Index(b0, b1, b2, b3, b4, b5, b6, b7, constants, P), V);
}

float4 ReadEmitterPosition(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                           device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, int I)
{
    return Read4(b6, I * 12);
}

float4 ReadEmitterVelocity(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                           device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, int I)
{
    return Read4(b6, I * 12 + 4);
}

float4 ReadEmitterScalar(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                         device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, int I)
{
    return Read4(b6, I * 12 + 8);
}

float4 SampleVelocity(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                      device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, float3 P)

{

    P = clamp(P, float3(0, 0, 0),
              float3(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).xyz - 1));
    int3 Base = int3(floor(P));
    float3 F = P - float3(Base);
    float4 Result = float4(0, 0, 0, 0);

    for (int Z = 0; Z < 2; ++Z)
        for (int Y = 0; Y < 2; ++Y)
            for (int X = 0; X < 2; ++X)

            {

                float W = (X != 0 ? F.x : 1 - F.x) * (Y != 0 ? F.y : 1 - F.y) * (Z != 0 ? F.z : 1 - F.z);

                Result +=
                    ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants,
                            min(Base + int3(X, Y, Z),
                                int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).xyz - 1)) *
                    W;
            }

    return Result;
}

float2 SampleScalar(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                    device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, float3 P,
                    bool Forward)

{

    P = clamp(P, float3(0, 0, 0),
              float3(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).xyz - 1));
    int3 Base = int3(floor(P));
    float3 F = P - float3(Base);
    float2 Result = float2(0, 0);

    for (int Z = 0; Z < 2; ++Z)
        for (int Y = 0; Y < 2; ++Y)
            for (int X = 0; X < 2; ++X)

            {

                float W = (X != 0 ? F.x : 1 - F.x) * (Y != 0 ? F.y : 1 - F.y) * (Z != 0 ? F.z : 1 - F.z);
                int3 Q = min(Base + int3(X, Y, Z),
                             int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).xyz - 1);

                Result += (Forward ? ReadSForward(b0, b1, b2, b3, b4, b5, b6, b7, constants, Q)
                                   : ReadSIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, Q)) *
                          W;
            }

    return Result;
}

bool Inside(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
            device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    return all(P >= 0) && all(P < int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).xyz);
}

bool Solid(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
           device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    return Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, P) &&
           ReadSolidIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P) != 0;
}

bool Closed(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
            device uint *b6, device uint *b7, constant uint4 *constants, int3 P)
{
    return P.x < 0 || P.x >= int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x ||
           P.y < 0 || P.z < 0 ||
           P.z >= int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z ||
           Solid(b0, b1, b2, b3, b4, b5, b6, b7, constants, P);
}

float3 Velocity(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
                device uint *b6, device uint *b7, constant uint4 *constants, float3 P)

{

    float3 V = float3(SampleVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - float3(0.5, 0, 0)).x,

                      SampleVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - float3(0, 0.5, 0)).y,

                      SampleVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - float3(0, 0, 0.5)).z);

    V *= saturate(P + 0.5);

    return V;
}

float3 Trace(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
             device uint *b6, device uint *b7, constant uint4 *constants, float3 P, float Dt)

{

    float3 Mid = P - Velocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, P) /
                         float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).xyz *
                         (0.5 * Dt);

    float3 End = P - Velocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, Mid) /
                         float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).xyz * Dt;

    if (Solid(b0, b1, b2, b3, b4, b5, b6, b7, constants, (int3)floor(End + 0.5)))

    {

        float3 Start = P;

        for (int I = 0; I < 5; ++I)

        {

            float3 Test = (Start + End) * 0.5;

            if (Solid(b0, b1, b2, b3, b4, b5, b6, b7, constants, (int3)floor(Test + 0.5)))
                End = Test;
            else
                Start = Test;
        }

        End = Start;
    }

    return End;
}

float3 BoundaryVelocity(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                        device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, int3 P, float3 V)

{

    if (Solid(b0, b1, b2, b3, b4, b5, b6, b7, constants, P))
        return ReadWallIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P).xyz;

    if (Closed(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + int3(1, 0, 0)))
        V.x = ReadWallIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P).x;

    if (Closed(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + int3(0, 1, 0)))
        V.y = ReadWallIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P).y;

    if (Closed(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + int3(0, 0, 1)))
        V.z = ReadWallIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P).z;

    return V;
}

kernel void Boundary(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                     device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                     device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                     constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                     uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                     uint3 gid [[threadgroup_position_in_grid]])

{

    if (tid.x >= uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        return;

    uint3 Id = uint3(tid.x % int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x,
                     tid.x / int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x %
                         int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y,
                     tid.x / (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y));

    if (!Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
        return;

    WriteVOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id),
              float4(BoundaryVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id),
                                      ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)).xyz),
                     0));

    WriteSOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id),
              Solid(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id))
                  ? float2(0, 0)
                  : ReadSIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)));
}

kernel void AdvectVelocity(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                           device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                           device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                           constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                           uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                           uint3 gid [[threadgroup_position_in_grid]])

{

    if (tid.x >= uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        return;

    uint3 Id = uint3(tid.x % int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x,
                     tid.x / int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x %
                         int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y,
                     tid.x / (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y));

    if (!Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
        return;

    float3 P = float3(Id);

    float3 X = Trace(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + float3(0.5, 0, 0),
                     float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).w);

    float3 Y = Trace(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + float3(0, 0.5, 0),
                     float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).w);

    float3 Z = Trace(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + float3(0, 0, 0.5),
                     float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).w);

    float3 V = float3(Velocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, X).x,
                      Velocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, Y).y,
                      Velocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, Z).z);

    WriteVOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id),
              float4(BoundaryVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id), V), 0));
}

kernel void Emit(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                 device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                 device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]], constant uint4 *constants [[buffer(8)]],
                 threadgroup float *shared0 [[threadgroup(0)]], uint3 tid [[thread_position_in_grid]],
                 uint3 lid [[thread_position_in_threadgroup]], uint3 gid [[threadgroup_position_in_grid]])

{

    if (tid.x >= uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        return;

    uint3 Id = uint3(tid.x % int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x,
                     tid.x / int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x %
                         int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y,
                     tid.x / (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y));

    if (!Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
        return;

    float3 V = ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)).xyz;

    float2 S = ReadSIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id));

    float3 P = float4(PF(constants, 8), PF(constants, 9), PF(constants, 10), PF(constants, 11)).xyz +
               (float3(Id) + 0.5) * float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).xyz;

    for (int I = 0; I < (int)float4(PF(constants, 16), PF(constants, 17), PF(constants, 18), PF(constants, 19)).z; ++I)

    {

        float R = length(P - ReadEmitterPosition(b0, b1, b2, b3, b4, b5, b6, b7, constants, I).xyz) /
                  ReadEmitterPosition(b0, b1, b2, b3, b4, b5, b6, b7, constants, I).w;

        if (R >= 1)
            continue;

        float W = 1 - R * R * (3 - 2 * R);

        float Pulse =
            0.92 + 0.08 * sin(float4(PF(constants, 8), PF(constants, 9), PF(constants, 10), PF(constants, 11)).w * 3.1);

        S = max(S, ReadEmitterScalar(b0, b1, b2, b3, b4, b5, b6, b7, constants, I).xy * W * float2(Pulse, 1));

        float3 Jet =
            ReadEmitterVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, I).xyz +
            float3(0.16 * sin(float4(PF(constants, 8), PF(constants, 9), PF(constants, 10), PF(constants, 11)).w * 2.7 +
                              P.z * 13),

                   0.08 * sin(float4(PF(constants, 8), PF(constants, 9), PF(constants, 10), PF(constants, 11)).w * 4.1 +
                              P.x * 11),
                   0.16 * cos(float4(PF(constants, 8), PF(constants, 9), PF(constants, 10), PF(constants, 11)).w * 3.3 +
                              P.x * 12));

        V += (Jet - V) *
             (1 - exp(-18 * W * float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).w));
    }

    WriteVOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id),
              float4(BoundaryVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id), V), 0));

    WriteSOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id),
              Solid(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)) ? float2(0, 0) : S);
}

float3 CenterVelocity(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4,
                      device uint *b5, device uint *b6, device uint *b7, constant uint4 *constants, int3 P)

{

    P = clamp(P, 0, int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).xyz - 1);

    return 0.5 * (ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P).xyz +
                  float3(P.x > 0 ? ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - int3(1, 0, 0)).x : 0,

                         P.y > 0 ? ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - int3(0, 1, 0)).y : 0,
                         P.z > 0 ? ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - int3(0, 0, 1)).z : 0));
}

kernel void Curl(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                 device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                 device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]], constant uint4 *constants [[buffer(8)]],
                 threadgroup float *shared0 [[threadgroup(0)]], uint3 tid [[thread_position_in_grid]],
                 uint3 lid [[thread_position_in_threadgroup]], uint3 gid [[threadgroup_position_in_grid]])

{

    if (tid.x >= uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        return;

    uint3 Id = uint3(tid.x % int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x,
                     tid.x / int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x %
                         int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y,
                     tid.x / (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y));

    if (!Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
        return;

    int3 P = int3(Id);

    float3 X = (CenterVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + int3(1, 0, 0)) -
                CenterVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - int3(1, 0, 0))) /
               (2 * float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).x);

    float3 Y = (CenterVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + int3(0, 1, 0)) -
                CenterVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - int3(0, 1, 0))) /
               (2 * float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).y);

    float3 Z = (CenterVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + int3(0, 0, 1)) -
                CenterVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - int3(0, 0, 1))) /
               (2 * float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).z);

    float3 C = float3(Y.z - Z.y, Z.x - X.z, X.y - Y.x);

    WriteVOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, P, float4(C, length(C)));
}

float3 Force(device uint *b0, device uint *b1, device uint *b2, device uint *b3, device uint *b4, device uint *b5,
             device uint *b6, device uint *b7, constant uint4 *constants, int3 P)

{

    P = clamp(P, 0, int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).xyz - 1);

    if (Solid(b0, b1, b2, b3, b4, b5, b6, b7, constants, P))
        return float3(0, 0, 0);

    float3 G =
        float3(ReadCurlIn(b0, b1, b2, b3, b4, b5, b6, b7, constants,
                          min(P + int3(1, 0, 0),
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).xyz - 1))
                       .w -
                   ReadCurlIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, max(P - int3(1, 0, 0), 0)).w,

               ReadCurlIn(b0, b1, b2, b3, b4, b5, b6, b7, constants,
                          min(P + int3(0, 1, 0),
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).xyz - 1))
                       .w -
                   ReadCurlIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, max(P - int3(0, 1, 0), 0)).w,

               ReadCurlIn(b0, b1, b2, b3, b4, b5, b6, b7, constants,
                          min(P + int3(0, 0, 1),
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).xyz - 1))
                       .w -
                   ReadCurlIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, max(P - int3(0, 0, 1), 0)).w) /
        float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).xyz;

    float3 F = cross(G / max(length(G), 1e-5), ReadCurlIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P).xyz) *
               float4(PF(constants, 12), PF(constants, 13), PF(constants, 14), PF(constants, 15)).z *
               min(float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).x,
                   min(float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).y,
                       float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).z));

    F.y += float4(PF(constants, 12), PF(constants, 13), PF(constants, 14), PF(constants, 15)).x *
               ReadSIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P).y -
           float4(PF(constants, 12), PF(constants, 13), PF(constants, 14), PF(constants, 15)).y *
               ReadSIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P).x;

    return F;
}

kernel void ApplyForces(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                        device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                        device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                        constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                        uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                        uint3 gid [[threadgroup_position_in_grid]])

{

    if (tid.x >= uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        return;

    uint3 Id = uint3(tid.x % int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x,
                     tid.x / int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x %
                         int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y,
                     tid.x / (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y));

    if (!Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
        return;

    int3 P = int3(Id);

    float3 F = 0.5 * (Force(b0, b1, b2, b3, b4, b5, b6, b7, constants, P) +
                      float3(Force(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + int3(1, 0, 0)).x,
                             Force(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + int3(0, 1, 0)).y,
                             Force(b0, b1, b2, b3, b4, b5, b6, b7, constants, P + int3(0, 0, 1)).z));

    WriteVOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, P,
              float4(BoundaryVelocity(
                         b0, b1, b2, b3, b4, b5, b6, b7, constants, P,
                         ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P).xyz +
                             F * float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).w),
                     0));
}

kernel void Divergence(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                       device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                       device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                       constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                       uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                       uint3 gid [[threadgroup_position_in_grid]])

{

    if (tid.x >= uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        return;

    uint3 Id = uint3(tid.x % int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x,
                     tid.x / int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x %
                         int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y,
                     tid.x / (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y));

    if (!Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
        return;

    int3 P = int3(Id);

    float3 Low = float3(P.x > 0 ? ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - int3(1, 0, 0)).x : 0,

                        P.y > 0 ? ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - int3(0, 1, 0)).y : 0,
                        P.z > 0 ? ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P - int3(0, 0, 1)).z : 0);

    WriteDivergenceOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, P,
                       Solid(b0, b1, b2, b3, b4, b5, b6, b7, constants, P)
                           ? 0
                           : dot((ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P).xyz - Low) /
                                     float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).xyz,
                                 float3(1, 1, 1)));
}

kernel void Pressure(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                     device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                     device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                     constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                     uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                     uint3 gid [[threadgroup_position_in_grid]])

{

    if (tid.x >= uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        return;

    uint3 Id = uint3(tid.x % int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x,
                     tid.x / int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x %
                         int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y,
                     tid.x / (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y));

    if (!Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)) ||
        ((Id.x + Id.y + Id.z) & 1) !=
            (uint)int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).w ||
        Solid(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
        return;

    int3 P = int3(Id);

    float Sum = 0, Diagonal = 0;

    for (int A = 0; A < 3; ++A)

        for (int Sign = -1; Sign <= 1; Sign += 2)

        {

            int3 Q = P;
            Q[A] += Sign;

            if (Closed(b0, b1, b2, b3, b4, b5, b6, b7, constants, Q))
                continue;

            float W = 1 / (float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7))[A] *
                           float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7))[A]);

            Diagonal += W;

            if (Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, Q))
                Sum += W * ReadPressureOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, Q);
        }

    if (Diagonal > 0)
        WritePressureOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, P,
                         ReadPressureOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, P) +
                             (float4(PF(constants, 12), PF(constants, 13), PF(constants, 14), PF(constants, 15)).w *
                              ((Sum - ReadDivergenceIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P)) / Diagonal -
                               ReadPressureOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, P))));
}

kernel void Project(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                    device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                    device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                    constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                    uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                    uint3 gid [[threadgroup_position_in_grid]])

{

    if (tid.x >= uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        return;

    uint3 Id = uint3(tid.x % int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x,
                     tid.x / int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x %
                         int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y,
                     tid.x / (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y));

    if (!Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
        return;

    int3 P = int3(Id);

    float3 V = ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P).xyz;

    for (int A = 0; A < 3; ++A)

    {

        int3 Q = P;
        Q[A] += 1;

        if (!Closed(b0, b1, b2, b3, b4, b5, b6, b7, constants, Q))
            V[A] -= ((Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, Q)
                          ? ReadPressureIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, Q)
                          : 0) -
                     ReadPressureIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, P)) /
                    float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7))[A];
    }

    WriteVOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, P,
              float4(BoundaryVelocity(b0, b1, b2, b3, b4, b5, b6, b7, constants, P, V), 0));
}

kernel void AdvectScalar(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                         device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                         device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                         constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                         uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                         uint3 gid [[threadgroup_position_in_grid]])

{

    if (tid.x >= uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        return;

    uint3 Id = uint3(tid.x % int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x,
                     tid.x / int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x %
                         int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y,
                     tid.x / (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y));

    if (!Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
        return;

    WriteSOut(
        b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id),
        Solid(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id))
            ? float2(0, 0)
            : SampleScalar(b0, b1, b2, b3, b4, b5, b6, b7, constants,
                           Trace(b0, b1, b2, b3, b4, b5, b6, b7, constants, float3(Id),
                                 float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).w),
                           false));
}

kernel void CorrectScalar(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                          device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                          device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                          constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                          uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                          uint3 gid [[threadgroup_position_in_grid]])

{

    if (tid.x >= uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        return;

    uint3 Id = uint3(tid.x % int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x,
                     tid.x / int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x %
                         int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y,
                     tid.x / (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y));

    if (!Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
        return;

    if (Solid(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
    {
        WriteSOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id), float2(0, 0));
        return;
    }

    float3 Back = Trace(b0, b1, b2, b3, b4, b5, b6, b7, constants, float3(Id),
                        float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).w),
           Forward = Trace(b0, b1, b2, b3, b4, b5, b6, b7, constants, float3(Id),
                           -float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).w);

    float2 Lo = float2(1e30, 1e30), Hi = float2(-1e30, -1e30);

    int3 Base = (int3)floor(Back);

    for (int Z = 0; Z < 2; ++Z)

        for (int Y = 0; Y < 2; ++Y)

            for (int X = 0; X < 2; ++X)

            {

                float2 S = ReadSIn(
                    b0, b1, b2, b3, b4, b5, b6, b7, constants,
                    clamp(Base + int3(X, Y, Z), 0,
                          int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).xyz - 1));

                Lo = min(Lo, S);
                Hi = max(Hi, S);
            }

    float2 Result = clamp(ReadSForward(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)) +
                              0.5 * (ReadSIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)) -
                                     SampleScalar(b0, b1, b2, b3, b4, b5, b6, b7, constants, Forward, true)),
                          Lo, Hi);

    float Fade =
        saturate((Id.y - (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y - 7.0)) / 6);

    WriteSOut(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id),
              Result * exp(-float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).w *
                           (float4(PF(constants, 16), PF(constants, 17), PF(constants, 18), PF(constants, 19)).xy +
                            Fade * 4)));
}

kernel void Impulse(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                    device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                    device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                    constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                    uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                    uint3 gid [[threadgroup_position_in_grid]])

{

    if (tid.x >= uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        return;

    uint3 Id = uint3(tid.x % int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x,
                     tid.x / int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x %
                         int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y,
                     tid.x / (int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                              int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y));

    if (!Inside(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)))
        return;

    float3 P = float4(PF(constants, 8), PF(constants, 9), PF(constants, 10), PF(constants, 11)).xyz +
               (float3(Id) + 0.5) * float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).xyz;

    float W =
        max(0, 1 - dot(P - float4(PF(constants, 20), PF(constants, 21), PF(constants, 22), PF(constants, 23)).xyz,
                       P - float4(PF(constants, 20), PF(constants, 21), PF(constants, 22), PF(constants, 23)).xyz) /
                       (float4(PF(constants, 20), PF(constants, 21), PF(constants, 22), PF(constants, 23)).w *
                        float4(PF(constants, 20), PF(constants, 21), PF(constants, 22), PF(constants, 23)).w));

    WriteVOut(
        b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id),
        float4(BoundaryVelocity(
                   b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id),
                   ReadVIn(b0, b1, b2, b3, b4, b5, b6, b7, constants, int3(Id)).xyz +
                       float4(PF(constants, 24), PF(constants, 25), PF(constants, 26), PF(constants, 27)).xyz * W * W),
               0));
}

kernel void ClearPressure(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                          device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                          device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                          constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                          uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                          uint3 gid [[threadgroup_position_in_grid]])
{
    if (tid.x < uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                     int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                     int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z))
        WriteFloat(b3, tid.x, 0);
}

kernel void MaxRate(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                    device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                    device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                    constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                    uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                    uint3 gid [[threadgroup_position_in_grid]])

{

    uint Count = uint(int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).x *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).y *
                      int4(PI(constants, 0), PI(constants, 1), PI(constants, 2), PI(constants, 3)).z);
    float Rate = 0;

    if (tid.x < Count)

    {

        float3 V = abs(Read4(b0, PI(constants, 28) + 4 * tid.x).xyz) /
                   float4(PF(constants, 4), PF(constants, 5), PF(constants, 6), PF(constants, 7)).xyz;

        Rate = max(V.x, max(V.y, V.z));
    }

    shared0[lid.x] = Rate;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    for (uint Stride = 128; Stride > 0; Stride >>= 1)

    {

        if (lid.x < Stride)
            shared0[lid.x] = max(shared0[lid.x], shared0[lid.x + Stride]);

        threadgroup_barrier(mem_flags::mem_threadgroup);
    }

    if (lid.x == 0)
        WriteFloat(b7, gid.x, shared0[0]);
}
)SHADER"
