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

struct Body

{

    float4 Position;

    float4 Rotation;

    float4 LinearVelocity;

    float4 AngularVelocity;

    float4 InverseInertia0;

    float4 InverseInertia1;

    float4 InverseInertia2;

    uint4 Flags;
};

struct Contact

{

    uint4 Bodies;

    float4 AnchorA;

    float4 AnchorB;

    float4 Normal;

    float4 Tangent;

    float4 Binormal;

    float4 Lambda;

    float4 Material;
};

struct Parameters

{

    float DeltaTime, Alpha, PenetrationSlop, MaxPositionCorrection, MaxAngularCorrection;
};

float4 ReadVector(device uint *B, uint I)

{

    return float4(ReadFloat(B, I), ReadFloat(B, I + 1), ReadFloat(B, I + 2), ReadFloat(B, I + 3));
}

void WriteVector(device uint *B, uint I, float4 V)

{

    WriteFloat(B, I, V.x);
    WriteFloat(B, I + 1, V.y);
    WriteFloat(B, I + 2, V.z);
    WriteFloat(B, I + 3, V.w);
}

uint4 ReadIndices(device uint *B, uint I)

{

    return uint4(ReadUInt(B, I), ReadUInt(B, I + 1), ReadUInt(B, I + 2), ReadUInt(B, I + 3));
}

void WriteIndices(device uint *B, uint I, uint4 V)

{

    WriteUInt(B, I, V.x);
    WriteUInt(B, I + 1, V.y);
    WriteUInt(B, I + 2, V.z);
    WriteUInt(B, I + 3, V.w);
}

Body ReadBody(device uint *B, uint I)

{

    I *= 32;
    Body V;

    V.Position = ReadVector(B, I + 0);

    V.Rotation = ReadVector(B, I + 4);

    V.LinearVelocity = ReadVector(B, I + 8);

    V.AngularVelocity = ReadVector(B, I + 12);

    V.InverseInertia0 = ReadVector(B, I + 16);

    V.InverseInertia1 = ReadVector(B, I + 20);

    V.InverseInertia2 = ReadVector(B, I + 24);

    V.Flags = ReadIndices(B, I + 28);

    return V;
}

void WriteBody(device uint *B, uint I, Body V)

{

    I *= 32;

    WriteVector(B, I + 0, V.Position);

    WriteVector(B, I + 4, V.Rotation);

    WriteVector(B, I + 8, V.LinearVelocity);

    WriteVector(B, I + 12, V.AngularVelocity);

    WriteVector(B, I + 16, V.InverseInertia0);

    WriteVector(B, I + 20, V.InverseInertia1);

    WriteVector(B, I + 24, V.InverseInertia2);

    WriteIndices(B, I + 28, V.Flags);
}

Contact ReadContact(device uint *B, uint I)

{

    I *= 32;
    Contact V;

    V.Bodies = ReadIndices(B, I + 0);

    V.AnchorA = ReadVector(B, I + 4);

    V.AnchorB = ReadVector(B, I + 8);

    V.Normal = ReadVector(B, I + 12);

    V.Tangent = ReadVector(B, I + 16);

    V.Binormal = ReadVector(B, I + 20);

    V.Lambda = ReadVector(B, I + 24);

    V.Material = ReadVector(B, I + 28);

    return V;
}

void WriteContact(device uint *B, uint I, Contact V)

{

    I *= 32;

    WriteIndices(B, I + 0, V.Bodies);

    WriteVector(B, I + 4, V.AnchorA);

    WriteVector(B, I + 8, V.AnchorB);

    WriteVector(B, I + 12, V.Normal);

    WriteVector(B, I + 16, V.Tangent);

    WriteVector(B, I + 20, V.Binormal);

    WriteVector(B, I + 24, V.Lambda);

    WriteVector(B, I + 28, V.Material);
}

float Dot3(float3 A, float3 B)

{

    float Result = (A.x * B.x + A.y * B.y) + A.z * B.z;

    return Result;
}

float3 Cross3(float3 A, float3 B)

{

    float3 Result = float3(A.y * B.z - A.z * B.y,

                           A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x);

    return Result;
}

float3 RotateVector(float4 Q, float3 V)

{

    float3 T = Cross3(Q.xyz, V) * 2.0f;

    float3 Result = (V + T * Q.w) + Cross3(Q.xyz, T);

    return Result;
}

float4 MultiplyRotation(float4 A, float4 B)

{

    float3 Vector = (Cross3(A.xyz, B.xyz) + A.xyz * B.w) + B.xyz * A.w;

    float Scalar = ((A.w * B.w - A.x * B.x) - A.y * B.y) - A.z * B.z;

    return float4(Vector, Scalar);
}

float3 InverseInertia(Body B, float3 V)

{

    if (B.Flags.x == 0)
        return float3(0, 0, 0);

    float3 Local = RotateVector(float4(-B.Rotation.xyz, B.Rotation.w), V);

    float3 Result = float3(Dot3(B.InverseInertia0.xyz, Local),

                           Dot3(B.InverseInertia1.xyz, Local), Dot3(B.InverseInertia2.xyz, Local));

    return RotateVector(B.Rotation, Result);
}

float EffectiveMass(Body A, Body B, float3 RA, float3 RB, float3 Axis)

{

    float3 AngularA = Cross3(RA, Axis), AngularB = Cross3(RB, Axis);

    float Result = A.Position.w + B.Position.w + Dot3(AngularA, InverseInertia(A, AngularA))

                   + Dot3(AngularB, InverseInertia(B, AngularB));

    return Result;
}

float3 PointVelocity(Body B, float3 Offset)

{

    float3 Result = B.Flags.y != 0 ? B.LinearVelocity.xyz + Cross3(B.AngularVelocity.xyz, Offset) : float3(0, 0, 0);

    return Result;
}

void PositionImpulse(thread Body &B, float3 Impulse, float3 Offset)

{

    if (B.Flags.x == 0 || Dot3(Impulse, Impulse) <= 1.0e-20f)
        return;

    float3 Angular = InverseInertia(B, Cross3(Offset, Impulse));

    float3 Position = B.Position.xyz + Impulse * B.Position.w;

    B.Position.xyz = Position;

    float Angle = sqrt(Dot3(Angular, Angular));

    if (Angle > 1.0e-8f)

    {

        float3 Axis = float3(Angular.x / Angle, Angular.y / Angle, Angular.z / Angle);

        float AxisLength = sqrt(Dot3(Axis, Axis));

        Axis = float3(Axis.x / AxisLength, Axis.y / AxisLength, Axis.z / AxisLength);

        float Sine = sin(Angle * 0.5f);

        float4 Delta = float4(Axis * Sine, cos(Angle * 0.5f));

        float4 Result = MultiplyRotation(Delta, B.Rotation);

        float Length = sqrt(((Result.w * Result.w + Result.x * Result.x)

                             + Result.y * Result.y) +
                            Result.z * Result.z);

        B.Rotation = float4(Result.x / Length, Result.y / Length, Result.z / Length, Result.w / Length);
    }

    B.Flags.z = 1;
}

void VelocityImpulse(thread Body &B, float3 Impulse, float3 Offset)

{

    if (B.Flags.x == 0 || Dot3(Impulse, Impulse) <= 1.0e-20f)
        return;

    float3 Linear = B.LinearVelocity.xyz + Impulse * B.Position.w;

    float3 Angular = B.AngularVelocity.xyz + InverseInertia(B, Cross3(Offset, Impulse));

    B.LinearVelocity.xyz = Linear;

    B.AngularVelocity.xyz = Angular;

    B.Flags.z = 1;
}

void ProjectPosition(Parameters P, thread Body &A, thread Body &B, thread Contact &C, float3 RA, float3 RB)

{

    float Weight = EffectiveMass(A, B, RA, RB, C.Normal.xyz);

    if (!isfinite(Weight) || Weight <= 1.0e-9f)
        return;

    float3 PA = A.Position.xyz + RA, PB = B.Position.xyz + RB;

    float Slop = C.Bodies.z ? C.AnchorA.w : P.PenetrationSlop;

    float Error = Dot3(PB - PA, C.Normal.xyz) - C.Material.w + Slop;

    float Delta;

    if (P.Alpha == 0.0f)
        Delta = -Error / Weight;

    else

    {

        float Scale = max(Weight, P.Alpha);

        float Denominator = Weight / Scale + P.Alpha / Scale;

        Delta = -(Error / Scale) / Denominator - C.Lambda.x * ((P.Alpha / Scale) / Denominator);
    }

    Delta = max(-C.Lambda.x, Delta);

    if (!isfinite(Delta))
        return;

    float Translation = max(A.Position.w, B.Position.w) * abs(Delta);

    float3 AngularA = InverseInertia(A, Cross3(RA, C.Normal.xyz));

    float3 AngularB = InverseInertia(B, Cross3(RB, C.Normal.xyz));

    float Rotation = max(sqrt(Dot3(AngularA, AngularA)), sqrt(Dot3(AngularB, AngularB))) * abs(Delta);

    float Scale = 1.0f;

    if (Translation > P.MaxPositionCorrection)
        Scale = min(Scale, P.MaxPositionCorrection / Translation);

    if (Rotation > P.MaxAngularCorrection)
        Scale = min(Scale, P.MaxAngularCorrection / Rotation);

    Delta *= Scale;

    C.Lambda.x += Delta;

    PositionImpulse(A, -C.Normal.xyz * Delta, RA);

    PositionImpulse(B, C.Normal.xyz * Delta, RB);
}

void ProjectVelocity(Parameters P, thread Body &A, thread Body &B, thread Contact &C, float3 RA, float3 RB)

{

    float Weight = EffectiveMass(A, B, RA, RB, C.Normal.xyz);

    if (!isfinite(Weight) || Weight <= 1.0e-9f)
        return;

    float Separation = Dot3((B.Position.xyz + RB) - (A.Position.xyz + RA), C.Normal.xyz) - C.Material.w;

    bool OpenGap = C.Bodies.z != 0 && Separation > 0.0f;

    float TargetVelocity = OpenGap ? -Separation / P.DeltaTime : C.Material.x;

    float Speed = Dot3(PointVelocity(B, RB) - PointVelocity(A, RA), C.Normal.xyz);

    float PreviousImpulse = C.Lambda.x / P.DeltaTime + C.Lambda.y;

    float ImpulseLimit = min(A.LinearVelocity.w, B.LinearVelocity.w);

    float TotalImpulse = max(0.0f, min(ImpulseLimit,

                                       PreviousImpulse + (TargetVelocity - Speed) / Weight));

    float Delta = TotalImpulse - PreviousImpulse;

    C.Lambda.y += Delta;

    VelocityImpulse(A, -C.Normal.xyz * Delta, RA);

    VelocityImpulse(B, C.Normal.xyz * Delta, RB);

    if (C.Bodies.w != 0)
        return;

    float3 Relative = PointVelocity(B, RB) - PointVelocity(A, RA);

    float KT = EffectiveMass(A, B, RA, RB, C.Tangent.xyz);

    float KB = EffectiveMass(A, B, RA, RB, C.Binormal.xyz);

    float KTB = Dot3(Cross3(RA, C.Tangent.xyz), InverseInertia(A, Cross3(RA, C.Binormal.xyz)))

                + Dot3(Cross3(RB, C.Tangent.xyz), InverseInertia(B, Cross3(RB, C.Binormal.xyz)));

    float Determinant = KT * KB - KTB * KTB;

    if (!isfinite(Determinant) || Determinant <= 1.0e-12f)
        return;

    float VT = Dot3(Relative, C.Tangent.xyz), VB = Dot3(Relative, C.Binormal.xyz);

    float Tangent = C.Lambda.z - (KB * VT - KTB * VB) / Determinant;

    float Binormal = C.Lambda.w - (KT * VB - KTB * VT) / Determinant;

    float Magnitude = sqrt(Tangent * Tangent + Binormal * Binormal);

    if (Magnitude > (OpenGap ? 0.0f : C.Material.z) * TotalImpulse && Magnitude > 0.0f)

    {

        float Scale = min(1.0f, (OpenGap ? 0.0f : C.Material.y) * TotalImpulse / Magnitude);

        Tangent *= Scale;

        Binormal *= Scale;
    }

    float3 FrictionImpulse = C.Tangent.xyz * (Tangent - C.Lambda.z)

                             + C.Binormal.xyz * (Binormal - C.Lambda.w);

    C.Lambda.z = Tangent;

    C.Lambda.w = Binormal;

    VelocityImpulse(A, -FrictionImpulse, RA);

    VelocityImpulse(B, FrictionImpulse, RB);
}

kernel void SolveContacts(device uint *b0 [[buffer(0)]], device uint *b1 [[buffer(1)]], device uint *b2 [[buffer(2)]],
                          device uint *b3 [[buffer(3)]], device uint *b4 [[buffer(4)]], device uint *b5 [[buffer(5)]],
                          device uint *b6 [[buffer(6)]], device uint *b7 [[buffer(7)]],
                          constant uint4 *constants [[buffer(8)]], threadgroup float *shared0 [[threadgroup(0)]],
                          uint3 tid [[thread_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]],
                          uint3 gid [[threadgroup_position_in_grid]])

{

    uint3 Id = tid;

    uint RowStart = (uint)PI(constants, 5), RowCount = (uint)PI(constants, 6), Stage = (uint)PI(constants, 7);

    Parameters P;

    P.DeltaTime = PF(constants, 0);
    P.Alpha = PF(constants, 1);
    P.PenetrationSlop = PF(constants, 2);

    P.MaxPositionCorrection = PF(constants, 3);
    P.MaxAngularCorrection = PF(constants, 4);

    if (Id.x >= RowCount)
        return;

    uint Row = ReadUInt(b2, RowStart + Id.x);

    Contact C = ReadContact(b1, Row);

    Body A = ReadBody(b0, C.Bodies.x), B = ReadBody(b0, C.Bodies.y);

    float3 RA = RotateVector(A.Rotation, C.AnchorA.xyz);

    float3 RB = RotateVector(B.Rotation, C.AnchorB.xyz);

    if (Stage == 0)
        ProjectPosition(P, A, B, C, RA, RB);

    else if (Stage == 1)

    {

        float3 Impulse = C.Normal.xyz * (C.Lambda.x / P.DeltaTime);

        VelocityImpulse(A, -Impulse, RA);

        VelocityImpulse(B, Impulse, RB);
    }

    else
        ProjectVelocity(P, A, B, C, RA, RB);

    if (A.Flags.x != 0)
        WriteBody(b0, C.Bodies.x, A);

    if (B.Flags.x != 0)
        WriteBody(b0, C.Bodies.y, B);

    WriteContact(b1, Row, C);
}
)SHADER"
