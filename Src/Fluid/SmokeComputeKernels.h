// Embedded so smoke simulation does not depend on the process working directory.
// Cell coordinates denote centers; each velocity component lies on its positive face.
R"HLSL(
cbuffer Parameters : register(b0)
{
	int4 Size;
	float4 CellDt;
	float4 MinimumTime;
	float4 Forces;
	float4 Decay;
	float4 ImpulsePosition;
	float4 ImpulseChange;
	float4 EmitterPosition[8];
	float4 EmitterVelocity[8];
	float4 EmitterScalar[8];
};
Texture3D<float4> VIn : register(t0);
Texture3D<float2> SIn : register(t1);
Texture3D<float2> SForward : register(t2);
Texture3D<float4> CurlIn : register(t3);
Texture3D<float> PressureIn : register(t4);
Texture3D<uint> SolidIn : register(t5);
Texture3D<float> DivergenceIn : register(t6);
Texture3D<float4> WallIn : register(t7);
RWTexture3D<float4> VOut : register(u0);
RWTexture3D<float2> SOut : register(u1);
RWTexture3D<float> PressureOut : register(u2);
RWTexture3D<float> DivergenceOut : register(u3);
RWBuffer<uint> SpeedOut : register(u4);
SamplerState LinearClamp : register(s0);

bool Inside(int3 P) { return all(P >= 0) && all(P < Size.xyz); }
bool Solid(int3 P) { return Inside(P) && SolidIn[P] != 0; }
bool Closed(int3 P) { return P.x < 0 || P.x >= Size.x || P.y < 0 || P.z < 0 || P.z >= Size.z || Solid(P); }
float3 UV(float3 P) { return (P + 0.5) / Size.xyz; }

float3 Velocity(float3 P)
{
	float3 V = float3(VIn.SampleLevel(LinearClamp, UV(P - float3(0.5, 0, 0)), 0).x,
		VIn.SampleLevel(LinearClamp, UV(P - float3(0, 0.5, 0)), 0).y,
		VIn.SampleLevel(LinearClamp, UV(P - float3(0, 0, 0.5)), 0).z);
	// Interpolate toward the missing zero normal face at the low walls.
	V *= saturate(P + 0.5);
	return V;
}

float3 Trace(float3 P, float Dt)
{
	float3 Mid = P - Velocity(P) / CellDt.xyz * (0.5 * Dt);
	float3 End = P - Velocity(Mid) / CellDt.xyz * Dt;
	if (Solid((int3)floor(End + 0.5)))
	{
		float3 Start = P;
		[unroll] for (int I = 0; I < 5; ++I)
		{
			float3 Test = (Start + End) * 0.5;
			if (Solid((int3)floor(Test + 0.5))) End = Test; else Start = Test;
		}
		End = Start;
	}
	return End;
}

float3 BoundaryVelocity(int3 P, float3 V)
{
	if (Solid(P)) return WallIn[P].xyz;
	if (Closed(P + int3(1,0,0))) V.x = WallIn[P].x;
	if (Closed(P + int3(0,1,0))) V.y = WallIn[P].y;
	if (Closed(P + int3(0,0,1))) V.z = WallIn[P].z;
	return V;
}

[numthreads(4,4,4)]
void Boundary(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return;
	VOut[Id] = float4(BoundaryVelocity(Id, VIn[Id].xyz), 0);
	SOut[Id] = Solid(Id) ? 0 : SIn[Id];
}

[numthreads(4,4,4)]
void AdvectVelocity(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return;
	float3 P = Id;
	float3 X = Trace(P + float3(0.5,0,0), CellDt.w);
	float3 Y = Trace(P + float3(0,0.5,0), CellDt.w);
	float3 Z = Trace(P + float3(0,0,0.5), CellDt.w);
	float3 V = float3(Velocity(X).x, Velocity(Y).y, Velocity(Z).z);
	VOut[Id] = float4(BoundaryVelocity(Id, V), 0);
}

[numthreads(4,4,4)]
void Emit(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return;
	float3 V = VIn[Id].xyz;
	float2 S = SIn[Id];
	float3 P = MinimumTime.xyz + (Id + 0.5) * CellDt.xyz;
	[loop] for (int I = 0; I < (int)Decay.z; ++I)
	{
		float R = length(P - EmitterPosition[I].xyz) / EmitterPosition[I].w;
		if (R >= 1) continue;
		float W = 1 - R * R * (3 - 2 * R);
		float Pulse = 0.92 + 0.08 * sin(MinimumTime.w * 3.1);
		S = max(S, EmitterScalar[I].xy * W * float2(Pulse, 1));
		float3 Jet = EmitterVelocity[I].xyz + float3(0.16 * sin(MinimumTime.w * 2.7 + P.z * 13),
			0.08 * sin(MinimumTime.w * 4.1 + P.x * 11), 0.16 * cos(MinimumTime.w * 3.3 + P.x * 12));
		V += (Jet - V) * (1 - exp(-18 * W * CellDt.w));
	}
	VOut[Id] = float4(BoundaryVelocity(Id, V), 0);
	SOut[Id] = Solid(Id) ? 0 : S;
}

float3 CenterVelocity(int3 P)
{
	P = clamp(P, 0, Size.xyz - 1);
	return 0.5 * (VIn[P].xyz + float3(P.x > 0 ? VIn[P - int3(1,0,0)].x : 0,
		P.y > 0 ? VIn[P - int3(0,1,0)].y : 0, P.z > 0 ? VIn[P - int3(0,0,1)].z : 0));
}

[numthreads(4,4,4)]
void Curl(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return;
	int3 P = Id;
	float3 X = (CenterVelocity(P + int3(1,0,0)) - CenterVelocity(P - int3(1,0,0))) / (2 * CellDt.x);
	float3 Y = (CenterVelocity(P + int3(0,1,0)) - CenterVelocity(P - int3(0,1,0))) / (2 * CellDt.y);
	float3 Z = (CenterVelocity(P + int3(0,0,1)) - CenterVelocity(P - int3(0,0,1))) / (2 * CellDt.z);
	float3 C = float3(Y.z - Z.y, Z.x - X.z, X.y - Y.x);
	VOut[P] = float4(C, length(C));
}

float3 Force(int3 P)
{
	P = clamp(P, 0, Size.xyz - 1);
	if (Solid(P)) return 0;
	float3 G = float3(CurlIn[min(P + int3(1,0,0), Size.xyz - 1)].w - CurlIn[max(P - int3(1,0,0), 0)].w,
		CurlIn[min(P + int3(0,1,0), Size.xyz - 1)].w - CurlIn[max(P - int3(0,1,0), 0)].w,
		CurlIn[min(P + int3(0,0,1), Size.xyz - 1)].w - CurlIn[max(P - int3(0,0,1), 0)].w) / CellDt.xyz;
	float3 F = cross(G / max(length(G), 1e-5), CurlIn[P].xyz) * Forces.z * min(CellDt.x, min(CellDt.y, CellDt.z));
	F.y += Forces.x * SIn[P].y - Forces.y * SIn[P].x;
	return F;
}

[numthreads(4,4,4)]
void ApplyForces(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return;
	int3 P = Id;
	float3 F = 0.5 * (Force(P) + float3(Force(P + int3(1,0,0)).x, Force(P + int3(0,1,0)).y, Force(P + int3(0,0,1)).z));
	VOut[P] = float4(BoundaryVelocity(P, VIn[P].xyz + F * CellDt.w), 0);
}

[numthreads(4,4,4)]
void Divergence(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return;
	int3 P = Id;
	float3 Low = float3(P.x > 0 ? VIn[P - int3(1,0,0)].x : 0,
		P.y > 0 ? VIn[P - int3(0,1,0)].y : 0, P.z > 0 ? VIn[P - int3(0,0,1)].z : 0);
	DivergenceOut[P] = Solid(P) ? 0 : dot((VIn[P].xyz - Low) / CellDt.xyz, 1);
}

[numthreads(4,4,4)]
void Pressure(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id) || ((Id.x + Id.y + Id.z) & 1) != (uint)Size.w || Solid(Id)) return;
	int3 P = Id;
	float Sum = 0, Diagonal = 0;
	[unroll] for (int A = 0; A < 3; ++A)
	[unroll] for (int Sign = -1; Sign <= 1; Sign += 2)
	{
		int3 Q = P; Q[A] += Sign;
		if (Closed(Q)) continue;
		float W = 1 / (CellDt[A] * CellDt[A]);
		Diagonal += W;
		if (Inside(Q)) Sum += W * PressureOut[Q];
	}
	if (Diagonal > 0) PressureOut[P] += Forces.w * ((Sum - DivergenceIn[P]) / Diagonal - PressureOut[P]);
}

[numthreads(4,4,4)]
void Project(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return;
	int3 P = Id;
	float3 V = VIn[P].xyz;
	[unroll] for (int A = 0; A < 3; ++A)
	{
		int3 Q = P; Q[A] += 1;
		if (!Closed(Q)) V[A] -= ((Inside(Q) ? PressureIn[Q] : 0) - PressureIn[P]) / CellDt[A];
	}
	VOut[P] = float4(BoundaryVelocity(P, V), 0);
}

[numthreads(4,4,4)]
void AdvectScalar(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return;
	SOut[Id] = Solid(Id) ? 0 : SIn.SampleLevel(LinearClamp, UV(Trace(Id, CellDt.w)), 0);
}

[numthreads(4,4,4)]
void CorrectScalar(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return;
	if (Solid(Id)) { SOut[Id] = 0; return; }
	float3 Back = Trace(Id, CellDt.w), Forward = Trace(Id, -CellDt.w);
	float2 Lo = 1e30, Hi = -1e30;
	int3 Base = (int3)floor(Back);
	[unroll] for (int Z = 0; Z < 2; ++Z)
	[unroll] for (int Y = 0; Y < 2; ++Y)
	[unroll] for (int X = 0; X < 2; ++X)
	{
		float2 S = SIn[clamp(Base + int3(X,Y,Z), 0, Size.xyz - 1)];
		Lo = min(Lo, S); Hi = max(Hi, S);
	}
	float2 Result = clamp(SForward[Id] + 0.5 * (SIn[Id] - SForward.SampleLevel(LinearClamp, UV(Forward), 0)), Lo, Hi);
	float Fade = saturate((Id.y - (Size.y - 7.0)) / 6);
	SOut[Id] = Result * exp(-CellDt.w * (Decay.xy + Fade * 4));
}

[numthreads(4,4,4)]
void Impulse(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return;
	float3 P = MinimumTime.xyz + (Id + 0.5) * CellDt.xyz;
	float W = max(0, 1 - dot(P - ImpulsePosition.xyz, P - ImpulsePosition.xyz) / (ImpulsePosition.w * ImpulsePosition.w));
	VOut[Id] = float4(BoundaryVelocity(Id, VIn[Id].xyz + ImpulseChange.xyz * W * W), 0);
}

groupshared float Rates[64];
[numthreads(4,4,4)]
void MaxRate(uint3 Id : SV_DispatchThreadID, uint Lane : SV_GroupIndex)
{
	float3 V = Inside(Id) ? abs(VIn[Id].xyz) / CellDt.xyz : 0;
	Rates[Lane] = max(V.x, max(V.y, V.z));
	GroupMemoryBarrierWithGroupSync();
	[unroll] for (uint Stride = 32; Stride > 0; Stride >>= 1)
	{
		if (Lane < Stride) Rates[Lane] = max(Rates[Lane], Rates[Lane + Stride]);
		GroupMemoryBarrierWithGroupSync();
	}
	if (Lane == 0) InterlockedMax(SpeedOut[0], asuint(Rates[0]));
}
)HLSL"
