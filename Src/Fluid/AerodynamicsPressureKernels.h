// Mass-aware pressure correction, kept on the simulation device.
R"HLSL(
cbuffer CouplingParameters : register(b0)
{
	int4 Size;
	float4 Weight;
	int4 Counts;
	float4 Relaxation;
};
struct Entry { int Index; float Weight; };
struct Face { int Cell; int Axis; int3 Nodes; float3 Weights; };
Texture3D<float4> State : register(t0);
Texture3D<float> Product : register(t1);
Texture3D<float> Previous : register(t2);
Texture3D<float4> Velocity : register(t3);
Texture3D<float4> Wall : register(t4);
Texture3D<uint> Solid : register(t5);
StructuredBuffer<int> RowOffsets : register(t6);
StructuredBuffer<Entry> RowEntries : register(t7);
StructuredBuffer<int2> CellRanges : register(t8);
StructuredBuffer<Entry> CellEntries : register(t9);
StructuredBuffer<float> Response : register(t10);
StructuredBuffer<Face> Faces : register(t11);
StructuredBuffer<uint> Lookup : register(t12);
StructuredBuffer<int3> ActiveCells : register(t13);
RWTexture3D<float4> StateOut : register(u0);
RWTexture3D<float> ScalarOut : register(u1);
RWStructuredBuffer<float> ResponseOut : register(u2);
RWStructuredBuffer<int2> CellRangesOut : register(u3);
RWStructuredBuffer<uint> LookupOut : register(u5);
RWTexture3D<float4> VelocityOut : register(u6);
RWTexture3D<float4> WallOut : register(u7);

int Count() { return Size.x * Size.y * Size.z; }
int3 Position(uint I) { return int3(I % Size.x, I / Size.x % Size.y, I / (Size.x * Size.y)); }
bool Inside(int3 P) { return all(P >= 0) && all(P < Size.xyz); }
bool Closed(int3 P) { return P.x < 0 || P.x >= Size.x || P.y < 0 || P.z < 0 || P.z >= Size.z || (Inside(P) && Solid[P] != 0); }
float Diagonal(int3 P)
{
	float D = 0;
	[unroll] for (int A = 0; A < 3; ++A)
	{
		int3 N = 0; N[A] = 1;
		D += Weight[A] * ((!Closed(P - N) ? 1 : 0) + (!Closed(P + N) ? 1 : 0));
	}
	return D;
}
float SurfaceProduct(uint I)
{
	float V = 0;
	for (int J = CellRanges[I].x; J < CellRanges[I].y; ++J) { Entry E = CellEntries[J]; V += E.Weight * Response[E.Index]; }
	return V;
}
[numthreads(512,1,1)]
void BuildLookup(uint I : SV_DispatchThreadID)
{
	if (I < Counts.x) { Face F = Faces[I]; LookupOut[F.Cell * 3 + F.Axis] = I; }
	if (I < Counts.y) { int3 C = ActiveCells[I]; CellRangesOut[C.x] = C.yz; }
}
[numthreads(512,1,1)]
void SurfaceResponse(uint I : SV_DispatchThreadID)
{
	if (I >= Size.w) return; float V = 0;
	for (int J = RowOffsets[I]; J < RowOffsets[I + 1]; ++J)
	{
		Entry E = RowEntries[J]; int3 P = Position(E.Index);
		float X = Product[P];
		V += E.Weight * X;
	}
	ResponseOut[I] = V;
}
[numthreads(8,4,4)]
void InitializeState(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return; int3 P = Id; uint I = Id.x + Size.x * (Id.y + Size.y * Id.z);
	float Base = Diagonal(P), D = Base, Divergence = 0;
	[unroll] for (int A = 0; A < 3; ++A)
	{
		int3 N = P; N[A] -= 1;
		Divergence += (Velocity[P][A] - (Inside(N) ? Velocity[N][A] : 0)) * sqrt(Weight[A]);
	}
	for (int J = CellRanges[I].x; J < CellRanges[I].y; ++J)
	{
		Entry E = CellEntries[J]; float Sum = 0;
		for (int K = RowOffsets[E.Index]; K < RowOffsets[E.Index + 1]; ++K) Sum += abs(RowEntries[K].Weight);
		D += abs(E.Weight) * Sum;
	}
	float R = Solid[P] != 0 || D <= 0 ? 0 : -Divergence;
	float Inv = Solid[P] != 0 || D <= 0 ? 0 : 1 / D;
	StateOut[P] = float4(R, Inv, Base, 0);
}
[numthreads(8,4,4)]
void RelaxPressure(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return; int3 P = Id; uint I = Id.x + Size.x * (Id.y + Size.y * Id.z); float4 S = State[P];
	float X = Product[P]; float V = S.z * X;
	[unroll] for (int A = 0; A < 3; ++A)
	{
		int3 N = 0; N[A] = 1;
		if (Inside(P - N) && !Closed(P - N)) V -= Weight[A] * Product[P - N];
		if (Inside(P + N) && !Closed(P + N)) V -= Weight[A] * Product[P + N];
	}
	float Step = Relaxation.x * S.y * (S.x - V - SurfaceProduct(I)) + Relaxation.y * (X - Previous[P]);
	ScalarOut[P] = X + Step;
}
[numthreads(8,4,4)]
void ApplyCorrection(uint3 Id : SV_DispatchThreadID)
{
	if (!Inside(Id)) return; int3 P = Id; uint I = Id.x + Size.x * (Id.y + Size.y * Id.z); float3 V = Velocity[P].xyz, W = Wall[P].xyz;
	[unroll] for (int A = 0; A < 3; ++A)
	{
		uint F = Lookup[I * 3 + A];
		if (F != 0xffffffff) { Face B = Faces[F]; W[A] += dot(B.Weights, float3(Response[B.Nodes.x], Response[B.Nodes.y], Response[B.Nodes.z])); }
		int3 N = P; N[A] += 1;
		if (Solid[P] || Closed(N)) V[A] = W[A];
		else V[A] -= ((Inside(N) ? Product[N] : 0) - Product[P]) * sqrt(Weight[A]);
	}
	ScalarOut[P] = Product[P];
	VelocityOut[P] = float4(V, 0); WallOut[P] = float4(W, 0);
}
)HLSL"
