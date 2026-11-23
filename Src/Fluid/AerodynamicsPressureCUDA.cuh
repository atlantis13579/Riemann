// Included after the MAC grid helpers in AerodynamicsBackendCUDA.cu.
template<class T> struct PressureBuffer
{
	T* Data = nullptr; size_t Capacity = 0;
	~PressureBuffer() { cudaFree(Data); }
	bool Upload(size_t Count, const T* Source = nullptr)
	{
		if (Count > Capacity)
		{
			cudaFree(Data); Data = nullptr; Capacity = 0;
			if (cudaMalloc(&Data, Count * sizeof(T)) != cudaSuccess) return false;
			Capacity = Count;
		}
		return !Source || !Count || cudaMemcpy(Data, Source, Count * sizeof(T), cudaMemcpyHostToDevice) == cudaSuccess;
	}
};
struct PressureData
{
	int Count, RowCount, FaceCount, GroupCount;
	int *Rows, *Cells, *Lookup;
	AerodynamicsPressureEntry *RowEntries, *CellEntries;
	AerodynamicsPressureFace* Faces;
	float *Response, *Product;
	float4 *State, *Next, *Partials, *Control;
};
__device__ int3 PressurePosition(const Grid& G, int I) { return make_int3(I % G.Size.x, I / G.Size.x % G.Size.y, I / (G.Size.x * G.Size.y)); }
__device__ float PressureDiagonal(const Grid& G, int3 P)
{
	float Result = 0;
#pragma unroll
	for (int A = 0; A < 3; ++A) for (int Sign = -1; Sign <= 1; Sign += 2)
		if (!G.Closed(Offset(P, A, Sign))) { const float H = Component(G.Cell, A); Result += 1 / (H * H); }
	return Result;
}
__device__ float SurfaceProduct(const PressureData& D, int I)
{
	float V = 0;
	for (int J = D.Cells[I]; J < D.Cells[I + 1]; ++J) { const auto E = D.CellEntries[J]; V += E.Weight * D.Response[E.Index]; }
	return V;
}
__device__ void PressureReduce(float4 V, float4* Partial)
{
	__shared__ float4 Scratch[256]; const int Lane = threadIdx.x;
	Scratch[Lane] = V; __syncthreads();
#pragma unroll
	for (int Stride = 128; Stride > 0; Stride >>= 1)
	{
		if (Lane < Stride) { Scratch[Lane].x += Scratch[Lane + Stride].x; Scratch[Lane].y += Scratch[Lane + Stride].y; Scratch[Lane].z = fmaxf(Scratch[Lane].z, Scratch[Lane + Stride].z); }
		__syncthreads();
	}
	if (Lane == 0) Partial[blockIdx.x] = Scratch[0];
}
__global__ void PressureLookup(PressureData D)
{
	const int I = blockIdx.x * blockDim.x + threadIdx.x; if (I >= D.FaceCount) return;
	const auto F = D.Faces[I]; D.Lookup[F.Cell * 3 + F.Axis] = I;
}
__global__ void PressureResponse(Grid G, PressureData D, int Phase)
{
	const int I = blockIdx.x * blockDim.x + threadIdx.x; if (I >= D.RowCount) return;
	float V = 0;
	for (int J = D.Rows[I]; J < D.Rows[I + 1]; ++J)
	{
		const auto E = D.RowEntries[J]; float X = Phase == 1 ? D.State[E.Index].z : G.Pressure[E.Index];
		if (Phase == 2) X += D.State[E.Index].x;
		V += E.Weight * X;
	}
	D.Response[I] = V;
}
__global__ void PressureInitialize(Grid G, PressureData D)
{
	const int I = blockIdx.x * blockDim.x + threadIdx.x; if (I >= D.Count) return;
	const int3 P = PressurePosition(G, I); float Diagonal = PressureDiagonal(G, P), Div = 0;
#pragma unroll
	for (int A = 0; A < 3; ++A)
	{
		const int3 N = Offset(P, A, -1);
		Div += (Component(XYZ(G.V[I]), A) - (G.Inside(N) ? Component(XYZ(G.V[G.Index(N)]), A) : 0)) / Component(G.Cell, A);
	}
	for (int J = D.Cells[I]; J < D.Cells[I + 1]; ++J) { const float W = D.CellEntries[J].Weight; Diagonal += W * W; }
	const float Inv = G.Solids[I] || Diagonal <= 0 ? 0 : 1 / Diagonal;
	const float R = Inv == 0 ? 0 : -Div - SurfaceProduct(D, I);
	D.State[I] = make_float4(0, R, R * Inv, Inv);
}
__global__ void PressureProduct(Grid G, PressureData D)
{
	const int I = blockIdx.x * blockDim.x + threadIdx.x; if (I >= D.Count) return;
	if (G.Solids[I]) { D.Product[I] = 0; return; }
	const int3 P = PressurePosition(G, I); float V = PressureDiagonal(G, P) * D.State[I].z;
#pragma unroll
	for (int A = 0; A < 3; ++A) for (int Sign = -1; Sign <= 1; Sign += 2)
	{
		const int3 N = Offset(P, A, Sign); const float H = Component(G.Cell, A);
		if (G.Inside(N) && !G.Closed(N)) V -= D.State[G.Index(N)].z / (H * H);
	}
	D.Product[I] = V + SurfaceProduct(D, I);
}
__global__ void PressureDirectionReduction(PressureData D)
{
	const int I = blockIdx.x * blockDim.x + threadIdx.x; float4 V = make_float4(0, 0, 0, 0);
	if (I < D.Count) { const float4 S = D.State[I]; V = make_float4(S.y * S.y * S.w, S.z * D.Product[I], fabsf(S.y), 0); }
	PressureReduce(V, D.Partials);
}
__global__ void PressureStepLength(PressureData D, bool First)
{
	float4 Sum = make_float4(0, 0, 0, 0);
	for (int I = threadIdx.x; I < D.GroupCount; I += 256) { const auto V = D.Partials[I]; Sum.x += V.x; Sum.y += V.y; Sum.z = fmaxf(Sum.z, V.z); }
	PressureReduce(Sum, D.Partials); if (threadIdx.x != 0) return; Sum = D.Partials[0];
	const float Tolerance = First ? fmaxf(1e-5f, Sum.z * 1e-4f) : D.Control[0].z;
	const float Alpha = Sum.z > Tolerance && Sum.x > 0 && Sum.y > 0 ? Sum.x / Sum.y : 0;
	D.Control[0] = make_float4(Alpha, Sum.x, Tolerance, Sum.z);
}
__global__ void PressureResidual(PressureData D)
{
	const int I = blockIdx.x * blockDim.x + threadIdx.x; float4 V = make_float4(0, 0, 0, 0);
	if (I < D.Count)
	{
		float4 S = D.State[I]; const float Alpha = D.Control[0].x;
		S.x += Alpha * S.z; S.y -= Alpha * D.Product[I]; D.Next[I] = S; V.x = S.y * S.y * S.w;
	}
	PressureReduce(V, D.Partials);
}
__global__ void PressureRatio(PressureData D)
{
	float4 V = make_float4(0, 0, 0, 0); for (int I = threadIdx.x; I < D.GroupCount; I += 256) V.x += D.Partials[I].x;
	PressureReduce(V, D.Partials); if (threadIdx.x != 0) return; const float Sum = D.Partials[0].x;
	float4 C = D.Control[0]; C.x = C.w > C.z && C.y > 0 ? Sum / C.y : 0; D.Control[0] = C;
}
__global__ void PressureDirection(PressureData D)
{
	const int I = blockIdx.x * blockDim.x + threadIdx.x; if (I >= D.Count) return;
	float4 S = D.Next[I]; S.z = S.y * S.w + D.Control[0].x * S.z; D.State[I] = S;
}
__global__ void PressureApply(Grid G, PressureData D)
{
	const int I = blockIdx.x * blockDim.x + threadIdx.x; if (I >= D.Count) return;
	const int3 P = PressurePosition(G, I); float3 V = XYZ(G.V[I]), W = XYZ(G.Wall[I]);
#pragma unroll
	for (int A = 0; A < 3; ++A)
	{
		const int F = D.Lookup[I * 3 + A]; float Wall = Component(W, A);
		if (F >= 0) { const auto B = D.Faces[F]; for (int J = 0; J < 3; ++J) Wall += B.Weights[J] * D.Response[B.Nodes[J]]; }
		const int3 N = Offset(P, A, 1);
		const float Velocity = G.Solids[I] || G.Closed(N) ? Wall : Component(V, A) - ((G.Inside(N) ? D.State[G.Index(N)].x : 0) - D.State[I].x) / Component(G.Cell, A);
		if (A == 0) { V.x = Velocity; W.x = Wall; } else if (A == 1) { V.y = Velocity; W.y = Wall; } else { V.z = Velocity; W.z = Wall; }
	}
	G.Pressure[I] += D.State[I].x; G.VOut[I] = XYZW(V); G.Wall[I] = XYZW(W);
}
