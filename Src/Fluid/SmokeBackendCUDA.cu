#include "SmokeGPUBackend.h"
#include "SmokeSimulation.h"
#include <cuda_runtime.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace Riemann
{
	namespace
	{
		__device__ float3 operator+(float3 A, float3 B)
		{
			return make_float3(A.x + B.x, A.y + B.y, A.z + B.z);
		}
		__device__ float3 operator-(float3 A, float3 B)
		{
			return make_float3(A.x - B.x, A.y - B.y, A.z - B.z);
		}
		__device__ float3 operator*(float3 A, float B)
		{
			return make_float3(A.x * B, A.y * B, A.z * B);
		}
		__device__ float3 operator/(float3 A, float3 B)
		{
			return make_float3(A.x / B.x, A.y / B.y, A.z / B.z);
		}
		__device__ float Dot(float3 A, float3 B)
		{
			return A.x * B.x + A.y * B.y + A.z * B.z;
		}
		__device__ float Length(float3 A)
		{
			return sqrtf(Dot(A, A));
		}
		__device__ float3 Cross(float3 A, float3 B)
		{
			return make_float3(A.y * B.z - A.z * B.y, A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x);
		}
		__device__ float Clamp(float V, float A, float B)
		{
			return fminf(fmaxf(V, A), B);
		}
		__device__ float3 XYZ(float4 V)
		{
			return make_float3(V.x, V.y, V.z);
		}
		__device__ float4 XYZW(float3 V, float W = 0)
		{
			return make_float4(V.x, V.y, V.z, W);
		}
		__device__ float Component(float3 V, int A)
		{
			return A == 0 ? V.x : A == 1 ? V.y : V.z;
		}
		__device__ int3 Offset(int3 P, int A, int Sign)
		{
			if (A == 0)
				P.x += Sign;
			else if (A == 1)
				P.y += Sign;
			else
				P.z += Sign;
			return P;
		}
		struct Emitter
		{
			float4 Position, Velocity, Scalar;
		};
		struct Grid
		{
			int3 Size;
			float3 Cell, Minimum;
			float Dt, Time, Buoyancy, Weight, Vorticity, Dissipation, Cooling, Relaxation;
			float4* V;
			float4* VOut;
			float4* Curl;
			float4* Wall;
			float2* S;
			float2* SOut;
			float2* Forward;
			float* Pressure;
			float* Divergence;
			unsigned char* Solids;
			Emitter* Emitters;
			int EmitterCount;
			__device__ bool Inside(int3 P) const
			{
				return P.x >= 0 && P.y >= 0 && P.z >= 0 && P.x < Size.x && P.y < Size.y && P.z < Size.z;
			}
			__device__ int Index(int3 P) const { return P.x + Size.x * (P.y + Size.y * P.z); }
			__device__ int3 Clamped(int3 P) const
			{
				return make_int3(max(0, min(P.x, Size.x - 1)), max(0, min(P.y, Size.y - 1)),
				                 max(0, min(P.z, Size.z - 1)));
			}
			__device__ bool Solid(int3 P) const { return Inside(P) && Solids[Index(P)] != 0; }
			__device__ bool Closed(int3 P) const
			{
				return P.x < 0 || P.x >= Size.x || P.y < 0 || P.z < 0 || P.z >= Size.z || Solid(P);
			}
		};
		__device__ int3 ThreadPosition()
		{
			return make_int3(blockIdx.x * blockDim.x + threadIdx.x, blockIdx.y * blockDim.y + threadIdx.y,
			                 blockIdx.z * blockDim.z + threadIdx.z);
		}
		__device__ float3 Position(int3 P)
		{
			return make_float3((float)P.x, (float)P.y, (float)P.z);
		}

		template <int A> __device__ float Face(const Grid& G, float3 P)
		{
			P = P - make_float3(A == 0 ? 0.5f : 0, A == 1 ? 0.5f : 0, A == 2 ? 0.5f : 0);
			P = make_float3(Clamp(P.x, 0, G.Size.x - 1), Clamp(P.y, 0, G.Size.y - 1), Clamp(P.z, 0, G.Size.z - 1));
			int3 B = make_int3((int)P.x, (int)P.y, (int)P.z);
			float3 T = P - Position(B);
			float Result = 0;
#pragma unroll
			for (int Z = 0; Z < 2; ++Z)
#pragma unroll
				for (int Y = 0; Y < 2; ++Y)
#pragma unroll
					for (int X = 0; X < 2; ++X)
					{
						float4 V = G.V[G.Index(G.Clamped(make_int3(B.x + X, B.y + Y, B.z + Z)))];
						float Value = A == 0 ? V.x : A == 1 ? V.y : V.z;
						Result += Value * (X ? T.x : 1 - T.x) * (Y ? T.y : 1 - T.y) * (Z ? T.z : 1 - T.z);
					}
			return Result;
		}
		__device__ float3 Velocity(const Grid& G, float3 P)
		{
			return make_float3(Face<0>(G, P) * Clamp(P.x + 0.5f, 0, 1), Face<1>(G, P) * Clamp(P.y + 0.5f, 0, 1),
			                   Face<2>(G, P) * Clamp(P.z + 0.5f, 0, 1));
		}
		__device__ float2 Scalar(const Grid& G, const float2* Field, float3 P)
		{
			int3 B = make_int3((int)floorf(P.x), (int)floorf(P.y), (int)floorf(P.z));
			float3 T = P - Position(B);
			float2 Result = make_float2(0, 0);
#pragma unroll
			for (int Z = 0; Z < 2; ++Z)
#pragma unroll
				for (int Y = 0; Y < 2; ++Y)
#pragma unroll
					for (int X = 0; X < 2; ++X)
					{
						float2 V = Field[G.Index(G.Clamped(make_int3(B.x + X, B.y + Y, B.z + Z)))];
						float W = (X ? T.x : 1 - T.x) * (Y ? T.y : 1 - T.y) * (Z ? T.z : 1 - T.z);
						Result.x += V.x * W;
						Result.y += V.y * W;
					}
			return Result;
		}
		__device__ bool SolidAt(const Grid& G, float3 P)
		{
			return G.Solid(make_int3((int)floorf(P.x + 0.5f), (int)floorf(P.y + 0.5f), (int)floorf(P.z + 0.5f)));
		}
		__device__ float3 Trace(const Grid& G, float3 P, float Dt)
		{
			float3 Mid = P - Velocity(G, P) / G.Cell * (0.5f * Dt);
			float3 End = P - Velocity(G, Mid) / G.Cell * Dt;
			if (SolidAt(G, End))
			{
				float3 Start = P;
#pragma unroll
				for (int I = 0; I < 5; ++I)
				{
					float3 Test = (Start + End) * 0.5f;
					if (SolidAt(G, Test))
						End = Test;
					else
						Start = Test;
				}
				End = Start;
			}
			return End;
		}
		__device__ float3 Boundaries(const Grid& G, int3 P, float3 V)
		{
			if (G.Solid(P))
				return XYZ(G.Wall[G.Index(P)]);
			if (G.Closed(Offset(P, 0, 1)))
				V.x = G.Wall[G.Index(P)].x;
			if (G.Closed(Offset(P, 1, 1)))
				V.y = G.Wall[G.Index(P)].y;
			if (G.Closed(Offset(P, 2, 1)))
				V.z = G.Wall[G.Index(P)].z;
			return V;
		}
		__global__ void Boundary(Grid G)
		{
			int3 P = ThreadPosition(); if (!G.Inside(P)) return;
			const int I = G.Index(P);
			G.VOut[I] = XYZW(Boundaries(G, P, XYZ(G.V[I])));
			G.SOut[I] = G.Solid(P) ? make_float2(0, 0) : G.S[I];
		}
		__global__ void AdvectVelocity(Grid G)
		{
			int3 P = ThreadPosition();
			if (!G.Inside(P))
				return;
			float3 F = Position(P);
			float3 X = Trace(G, F + make_float3(0.5f, 0, 0), G.Dt), Y = Trace(G, F + make_float3(0, 0.5f, 0), G.Dt),
			       Z = Trace(G, F + make_float3(0, 0, 0.5f), G.Dt);
			G.VOut[G.Index(P)] =
			    XYZW(Boundaries(G, P, make_float3(Velocity(G, X).x, Velocity(G, Y).y, Velocity(G, Z).z)));
		}
		__global__ void Emit(Grid G)
		{
			int3 P = ThreadPosition();
			if (!G.Inside(P))
				return;
			int I = G.Index(P);
			float3 W =
			    G.Minimum + make_float3((P.x + 0.5f) * G.Cell.x, (P.y + 0.5f) * G.Cell.y, (P.z + 0.5f) * G.Cell.z);
			float3 V = XYZ(G.V[I]);
			float2 S = G.S[I];
			for (int E = 0; E < G.EmitterCount; ++E)
			{
				Emitter Source = G.Emitters[E];
				float R = Length(W - XYZ(Source.Position)) / Source.Position.w;
				if (R >= 1)
					continue;
				float Weight = 1 - R * R * (3 - 2 * R), Pulse = 0.92f + 0.08f * sinf(G.Time * 3.1f);
				S.x = fmaxf(S.x, Source.Scalar.x * Weight * Pulse);
				S.y = fmaxf(S.y, Source.Scalar.y * Weight);
				float3 Jet = XYZ(Source.Velocity) + make_float3(0.16f * sinf(G.Time * 2.7f + W.z * 13),
				                                                0.08f * sinf(G.Time * 4.1f + W.x * 11),
				                                                0.16f * cosf(G.Time * 3.3f + W.x * 12));
				V = V + (Jet - V) * (1 - expf(-18 * Weight * G.Dt));
			}
			G.VOut[I] = XYZW(Boundaries(G, P, V));
			G.SOut[I] = G.Solid(P) ? make_float2(0, 0) : S;
		}
		__device__ float3 CenterVelocity(const Grid& G, int3 P)
		{
			P = G.Clamped(P);
			return (XYZ(G.V[G.Index(P)]) + make_float3(P.x > 0 ? G.V[G.Index(Offset(P, 0, -1))].x : 0,
			                                           P.y > 0 ? G.V[G.Index(Offset(P, 1, -1))].y : 0,
			                                           P.z > 0 ? G.V[G.Index(Offset(P, 2, -1))].z : 0)) *
			       0.5f;
		}
		__global__ void Curl(Grid G)
		{
			int3 P = ThreadPosition();
			if (!G.Inside(P))
				return;
			float3 X = (CenterVelocity(G, Offset(P, 0, 1)) - CenterVelocity(G, Offset(P, 0, -1))) * (0.5f / G.Cell.x);
			float3 Y = (CenterVelocity(G, Offset(P, 1, 1)) - CenterVelocity(G, Offset(P, 1, -1))) * (0.5f / G.Cell.y);
			float3 Z = (CenterVelocity(G, Offset(P, 2, 1)) - CenterVelocity(G, Offset(P, 2, -1))) * (0.5f / G.Cell.z);
			float3 C = make_float3(Y.z - Z.y, Z.x - X.z, X.y - Y.x);
			G.Curl[G.Index(P)] = XYZW(C, Length(C));
		}
		__device__ float3 Force(const Grid& G, int3 P)
		{
			P = G.Clamped(P);
			if (G.Solid(P))
				return make_float3(0, 0, 0);
			float3 Grad =
			    make_float3(
			        G.Curl[G.Index(G.Clamped(Offset(P, 0, 1)))].w - G.Curl[G.Index(G.Clamped(Offset(P, 0, -1)))].w,
			        G.Curl[G.Index(G.Clamped(Offset(P, 1, 1)))].w - G.Curl[G.Index(G.Clamped(Offset(P, 1, -1)))].w,
			        G.Curl[G.Index(G.Clamped(Offset(P, 2, 1)))].w - G.Curl[G.Index(G.Clamped(Offset(P, 2, -1)))].w) /
			    G.Cell;
			float3 F = Cross(Grad * (1 / fmaxf(Length(Grad), 1e-5f)), XYZ(G.Curl[G.Index(P)])) *
			           (G.Vorticity * fminf(G.Cell.x, fminf(G.Cell.y, G.Cell.z)));
			float2 S = G.S[G.Index(P)];
			F.y += G.Buoyancy * S.y - G.Weight * S.x;
			return F;
		}
		__global__ void ApplyForces(Grid G)
		{
			int3 P = ThreadPosition();
			if (!G.Inside(P))
				return;
			float3 F = (Force(G, P) + make_float3(Force(G, Offset(P, 0, 1)).x, Force(G, Offset(P, 1, 1)).y,
			                                      Force(G, Offset(P, 2, 1)).z)) *
			           0.5f;
			G.VOut[G.Index(P)] = XYZW(Boundaries(G, P, XYZ(G.V[G.Index(P)]) + F * G.Dt));
		}
		__global__ void Divergence(Grid G)
		{
			int3 P = ThreadPosition();
			if (!G.Inside(P))
				return;
			float3 Low = make_float3(P.x > 0 ? G.V[G.Index(Offset(P, 0, -1))].x : 0,
			                         P.y > 0 ? G.V[G.Index(Offset(P, 1, -1))].y : 0,
			                         P.z > 0 ? G.V[G.Index(Offset(P, 2, -1))].z : 0);
			float3 D = (XYZ(G.V[G.Index(P)]) - Low) / G.Cell;
			G.Divergence[G.Index(P)] = G.Solid(P) ? 0 : D.x + D.y + D.z;
		}
		__global__ void Pressure(Grid G, int Parity)
		{
			int3 P = ThreadPosition();
			if (!G.Inside(P) || ((P.x + P.y + P.z) & 1) != Parity || G.Solid(P))
				return;
			float Sum = 0, Diagonal = 0;
#pragma unroll
			for (int A = 0; A < 3; ++A)
#pragma unroll
				for (int Sign = -1; Sign <= 1; Sign += 2)
				{
					int3 Q = Offset(P, A, Sign);
					if (G.Closed(Q))
						continue;
					float H = Component(G.Cell, A), W = 1 / (H * H);
					Diagonal += W;
					if (G.Inside(Q))
						Sum += W * G.Pressure[G.Index(Q)];
				}
			int I = G.Index(P);
			if (Diagonal > 0)
				G.Pressure[I] += G.Relaxation * ((Sum - G.Divergence[I]) / Diagonal - G.Pressure[I]);
		}
		__global__ void Project(Grid G)
		{
			int3 P = ThreadPosition();
			if (!G.Inside(P))
				return;
			float3 V = XYZ(G.V[G.Index(P)]);
#pragma unroll
			for (int A = 0; A < 3; ++A)
			{
				int3 Q = Offset(P, A, 1);
				if (G.Closed(Q))
					continue;
				float Delta =
				    ((G.Inside(Q) ? G.Pressure[G.Index(Q)] : 0) - G.Pressure[G.Index(P)]) / Component(G.Cell, A);
				if (A == 0)
					V.x -= Delta;
				else if (A == 1)
					V.y -= Delta;
				else
					V.z -= Delta;
			}
			G.VOut[G.Index(P)] = XYZW(Boundaries(G, P, V));
		}
		__global__ void AdvectScalar(Grid G)
		{
			int3 P = ThreadPosition();
			if (!G.Inside(P))
				return;
			G.Forward[G.Index(P)] = G.Solid(P) ? make_float2(0, 0) : Scalar(G, G.S, Trace(G, Position(P), G.Dt));
		}
		__global__ void CorrectScalar(Grid G)
		{
			int3 P = ThreadPosition();
			if (!G.Inside(P))
				return;
			int I = G.Index(P);
			if (G.Solid(P))
			{
				G.SOut[I] = make_float2(0, 0);
				return;
			}
			float3 Back = Trace(G, Position(P), G.Dt), Forward = Trace(G, Position(P), -G.Dt);
			int3 Base = make_int3((int)floorf(Back.x), (int)floorf(Back.y), (int)floorf(Back.z));
			float2 Lo = make_float2(1e30f, 1e30f), Hi = make_float2(-1e30f, -1e30f);
#pragma unroll
			for (int Z = 0; Z < 2; ++Z)
#pragma unroll
				for (int Y = 0; Y < 2; ++Y)
#pragma unroll
					for (int X = 0; X < 2; ++X)
					{
						float2 S = G.S[G.Index(G.Clamped(make_int3(Base.x + X, Base.y + Y, Base.z + Z)))];
						Lo.x = fminf(Lo.x, S.x);
						Lo.y = fminf(Lo.y, S.y);
						Hi.x = fmaxf(Hi.x, S.x);
						Hi.y = fmaxf(Hi.y, S.y);
					}
			float2 R = Scalar(G, G.Forward, Forward), S = G.S[I], F = G.Forward[I];
			float Fade = Clamp((P.y - (G.Size.y - 7.0f)) / 6, 0, 1);
			G.SOut[I] =
			    make_float2(Clamp(F.x + 0.5f * (S.x - R.x), Lo.x, Hi.x) * expf(-G.Dt * (G.Dissipation + Fade * 4)),
			                Clamp(F.y + 0.5f * (S.y - R.y), Lo.y, Hi.y) * expf(-G.Dt * (G.Cooling + Fade * 4)));
		}
		__global__ void Impulse(Grid G, float3 Center, float Radius, float3 Change)
		{
			int3 P = ThreadPosition();
			if (!G.Inside(P))
				return;
			float3 W = G.Minimum +
			           make_float3((P.x + 0.5f) * G.Cell.x, (P.y + 0.5f) * G.Cell.y, (P.z + 0.5f) * G.Cell.z),
			       D = W - Center;
			float Weight = fmaxf(0, 1 - Dot(D, D) / (Radius * Radius));
			G.VOut[G.Index(P)] = XYZW(Boundaries(G, P, XYZ(G.V[G.Index(P)]) + Change * (Weight * Weight)));
		}
		__global__ void MaxRate(Grid G, unsigned* Result)
		{
			__shared__ float Rates[64];
			int3 P = ThreadPosition();
			int Lane = threadIdx.x + 4 * (threadIdx.y + 4 * threadIdx.z);
			float3 V = G.Inside(P) ? XYZ(G.V[G.Index(P)]) : make_float3(0, 0, 0);
			Rates[Lane] = fmaxf(fabsf(V.x) / G.Cell.x, fmaxf(fabsf(V.y) / G.Cell.y, fabsf(V.z) / G.Cell.z));
			__syncthreads();
#pragma unroll
			for (int Stride = 32; Stride > 0; Stride >>= 1)
			{
				if (Lane < Stride)
					Rates[Lane] = fmaxf(Rates[Lane], Rates[Lane + Stride]);
				__syncthreads();
			}
			if (Lane == 0)
				atomicMax(Result, __float_as_uint(Rates[0]));
		}

		class CUDASmoke final : public SmokeGPUBackend
		{
			Grid m_Grid = {};
			unsigned* m_Rate = nullptr;
			float m_MaxRate = 0;
			size_t m_Count = 0;
			int m_Iterations = 0;
			dim3 m_Blocks, m_Threads = dim3(4, 4, 4);
			std::vector<float2> m_HostScalar;
			mutable std::vector<float4> m_HostVelocity;

		  public:
			~CUDASmoke()
			{
				cudaFree(m_Grid.V);
				cudaFree(m_Grid.VOut);
				cudaFree(m_Grid.Curl);
				cudaFree(m_Grid.Wall);
				cudaFree(m_Grid.S);
				cudaFree(m_Grid.SOut);
				cudaFree(m_Grid.Forward);
				cudaFree(m_Grid.Pressure);
				cudaFree(m_Grid.Divergence);
				cudaFree(m_Grid.Solids);
				cudaFree(m_Grid.Emitters);
				cudaFree(m_Rate);
			}
			bool Initialize(const SmokeSimulationSettings& S, const std::vector<unsigned char>& Solid)
			{
				int Devices = 0;
				if (cudaGetDeviceCount(&Devices) != cudaSuccess || Devices == 0)
					return false;
				m_Count = Solid.size();
				m_Iterations = S.PressureIterations;
				m_Grid.Size = make_int3(S.Resolution.x, S.Resolution.y, S.Resolution.z);
				Vector3 Extent = S.BoundsMax - S.BoundsMin;
				m_Grid.Cell =
				    make_float3(Extent.x / S.Resolution.x, Extent.y / S.Resolution.y, Extent.z / S.Resolution.z);
				m_Grid.Minimum = make_float3(S.BoundsMin.x, S.BoundsMin.y, S.BoundsMin.z);
				m_Grid.Buoyancy = S.Buoyancy;
				const int ShortAxis = std::min(S.Resolution.x, std::min(S.Resolution.y, S.Resolution.z));
				m_Grid.Relaxation = std::max(1.75f, std::min(1.93f, 2.0f / (1.0f + std::sin(3.14159265f / ShortAxis))));
				m_Grid.Weight = S.SmokeWeight;
				m_Grid.Vorticity = S.Vorticity;
				m_Grid.Dissipation = S.DensityDissipation;
				m_Grid.Cooling = S.Cooling;
				m_Grid.EmitterCount = (int)S.Emitters.size();
				m_Blocks = dim3((S.Resolution.x + 3) / 4, (S.Resolution.y + 3) / 4, (S.Resolution.z + 3) / 4);
				if (cudaMalloc(&m_Grid.V, m_Count * sizeof(float4)) != cudaSuccess ||
				    cudaMalloc(&m_Grid.VOut, m_Count * sizeof(float4)) != cudaSuccess ||
				    cudaMalloc(&m_Grid.Curl, m_Count * sizeof(float4)) != cudaSuccess ||
				    cudaMalloc(&m_Grid.Wall, m_Count * sizeof(float4)) != cudaSuccess ||
				    cudaMalloc(&m_Grid.S, m_Count * sizeof(float2)) != cudaSuccess ||
				    cudaMalloc(&m_Grid.SOut, m_Count * sizeof(float2)) != cudaSuccess ||
				    cudaMalloc(&m_Grid.Forward, m_Count * sizeof(float2)) != cudaSuccess ||
				    cudaMalloc(&m_Grid.Pressure, m_Count * sizeof(float)) != cudaSuccess ||
				    cudaMalloc(&m_Grid.Divergence, m_Count * sizeof(float)) != cudaSuccess ||
				    cudaMalloc(&m_Grid.Solids, m_Count) != cudaSuccess ||
				    cudaMalloc(&m_Rate, sizeof(unsigned)) != cudaSuccess)
					return false;
				if (cudaMemcpy(m_Grid.Solids, Solid.data(), m_Count, cudaMemcpyHostToDevice) != cudaSuccess)
					return false;
				if (!S.Emitters.empty())
				{
					std::vector<Emitter> Sources;
					for (const SmokeEmitter& E : S.Emitters)
						Sources.push_back({make_float4(E.Position.x, E.Position.y, E.Position.z, E.Radius),
						                   make_float4(E.Velocity.x, E.Velocity.y, E.Velocity.z, 0),
						                   make_float4(E.Density, E.Temperature, 0, 0)});
					if (cudaMalloc(&m_Grid.Emitters, Sources.size() * sizeof(Emitter)) != cudaSuccess ||
					    cudaMemcpy(m_Grid.Emitters, Sources.data(), Sources.size() * sizeof(Emitter),
					               cudaMemcpyHostToDevice) != cudaSuccess)
						return false;
				}
				m_HostScalar.resize(m_Count);
				m_HostVelocity.resize(m_Count);
				cudaMemset(m_Grid.Wall, 0, m_Count * sizeof(float4));
				Reset();
				return cudaGetLastError() == cudaSuccess;
			}
			bool UpdateObstacles(const std::vector<unsigned char>& Solid, const std::vector<Vector3>& Velocity) override
			{
				for (size_t I = 0; I < m_Count; ++I)
				{
					const Vector3& V = Velocity[I];
					m_HostVelocity[I] = make_float4(V.x, V.y, V.z, 0);
					m_MaxRate = std::max(m_MaxRate, std::max(std::fabs(V.x) / m_Grid.Cell.x,
						std::max(std::fabs(V.y) / m_Grid.Cell.y, std::fabs(V.z) / m_Grid.Cell.z)));
				}
				if (cudaMemcpy(m_Grid.Solids, Solid.data(), m_Count, cudaMemcpyHostToDevice) != cudaSuccess
					|| cudaMemcpy(m_Grid.Wall, m_HostVelocity.data(), m_Count * sizeof(float4), cudaMemcpyHostToDevice) != cudaSuccess) return false;
				Boundary<<<m_Blocks, m_Threads>>>(m_Grid);
				std::swap(m_Grid.V, m_Grid.VOut); std::swap(m_Grid.S, m_Grid.SOut);
				return cudaGetLastError() == cudaSuccess;
			}

			void Reset() override
			{
				cudaMemset(m_Grid.V, 0, m_Count * sizeof(float4));
				cudaMemset(m_Grid.VOut, 0, m_Count * sizeof(float4));
				cudaMemset(m_Grid.S, 0, m_Count * sizeof(float2));
				cudaMemset(m_Grid.SOut, 0, m_Count * sizeof(float2));
				cudaMemset(m_Grid.Forward, 0, m_Count * sizeof(float2));
				m_MaxRate = 0;
			}
			void Step(float Dt, float Time) override
			{
				m_Grid.Dt = Dt;
				m_Grid.Time = Time;
				AdvectVelocity<<<m_Blocks, m_Threads>>>(m_Grid);
				std::swap(m_Grid.V, m_Grid.VOut);
				Emit<<<m_Blocks, m_Threads>>>(m_Grid);
				std::swap(m_Grid.V, m_Grid.VOut);
				std::swap(m_Grid.S, m_Grid.SOut);
				Curl<<<m_Blocks, m_Threads>>>(m_Grid);
				ApplyForces<<<m_Blocks, m_Threads>>>(m_Grid);
				std::swap(m_Grid.V, m_Grid.VOut);
				Divergence<<<m_Blocks, m_Threads>>>(m_Grid);
				cudaMemset(m_Grid.Pressure, 0, m_Count * sizeof(float));
				for (int I = 0; I < m_Iterations; ++I)
					for (int Parity = 0; Parity < 2; ++Parity)
						Pressure<<<m_Blocks, m_Threads>>>(m_Grid, Parity);
				Project<<<m_Blocks, m_Threads>>>(m_Grid);
				std::swap(m_Grid.V, m_Grid.VOut);
				AdvectScalar<<<m_Blocks, m_Threads>>>(m_Grid);
				CorrectScalar<<<m_Blocks, m_Threads>>>(m_Grid);
				std::swap(m_Grid.S, m_Grid.SOut);
			}
			void ApplyImpulse(const Vector3& Position, float Radius, const Vector3& Change) override
			{
				Impulse<<<m_Blocks, m_Threads>>>(m_Grid, make_float3(Position.x, Position.y, Position.z), Radius,
				                                 make_float3(Change.x, Change.y, Change.z));
				std::swap(m_Grid.V, m_Grid.VOut);
				m_MaxRate += Change.Length() / std::min(m_Grid.Cell.x, std::min(m_Grid.Cell.y, m_Grid.Cell.z));
			}
			bool Readback(std::vector<float>& Density, std::vector<float>& Temperature) override
			{
				if (cudaGetLastError() != cudaSuccess)
					return false;
				cudaMemset(m_Rate, 0, sizeof(unsigned));
				MaxRate<<<m_Blocks, m_Threads>>>(m_Grid, m_Rate);
				if (cudaMemcpy(m_HostScalar.data(), m_Grid.S, m_Count * sizeof(float2), cudaMemcpyDeviceToHost) !=
				        cudaSuccess ||
				    cudaMemcpy(&m_MaxRate, m_Rate, sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess)
					return false;
				for (size_t I = 0; I < m_Count; ++I)
				{
					Density[I] = m_HostScalar[I].x;
					Temperature[I] = m_HostScalar[I].y;
				}
				return true;
			}
			bool ReadVelocity(std::vector<Vector3>& Velocity) const override
			{
				if (cudaMemcpy(m_HostVelocity.data(), m_Grid.V, m_Count * sizeof(float4), cudaMemcpyDeviceToHost) !=
				    cudaSuccess)
					return false;
				for (size_t I = 0; I < m_Count; ++I)
					Velocity[I] = Vector3(m_HostVelocity[I].x, m_HostVelocity[I].y, m_HostVelocity[I].z);
				return true;
			}
			float GetMaxRate() const override { return m_MaxRate; }
			bool ReadPressure(std::vector<float>& Pressure) const override
			{
				return cudaMemcpy(Pressure.data(), m_Grid.Pressure, m_Count * sizeof(float), cudaMemcpyDeviceToHost) == cudaSuccess;
			}
		};
	} // namespace

	std::unique_ptr<SmokeGPUBackend> SmokeGPUBackend::CreateCUDA(const SmokeSimulationSettings& Settings,
	                                                             const std::vector<unsigned char>& Solid)
	{
		std::unique_ptr<CUDASmoke> Backend(new CUDASmoke());
		if (!Backend->Initialize(Settings, Solid))
			return nullptr;
		return std::move(Backend);
	}
} // namespace Riemann
