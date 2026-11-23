#include "AerodynamicsGPUBackend.h"
#include "AerodynamicsTypes.h"

#if defined(_WIN32) && defined(RIEMANN_ENABLE_COMPUTE_SHADER)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "../Maths/Vector4.h"

namespace Riemann
{
	namespace
	{
		using Microsoft::WRL::ComPtr;
		const char* AerodynamicsSource =
#include "AerodynamicsComputeKernels.h"
		;
		const char* PressureSource =
#include "AerodynamicsPressureKernels.h"
		;
		class ComputeAerodynamics final : public AerodynamicsGPUBackend
		{
			enum { AdvectV, EmitSource, Curl, Force, Divergence, Pressure, Project, AdvectS, CorrectS, Impulse, MaxRate, Boundary, KernelCount };
			struct Texture
			{
				ComPtr<ID3D11Texture3D> Data;
				ComPtr<ID3D11ShaderResourceView> SRV;
				ComPtr<ID3D11UnorderedAccessView> UAV;
			};
			struct Parameters
			{
				int Size[4];
				Vector4 CellDt, MinimumTime, Forces, Decay, ImpulsePosition, ImpulseChange;
				Vector4 EmitterPosition[8], EmitterVelocity[8], EmitterScalar[8];
			};
			ComPtr<ID3D11Device> m_Device;
			ComPtr<ID3D11DeviceContext> m_Context;
			ComPtr<ID3D11ComputeShader> m_Kernels[KernelCount];
			ComPtr<ID3D11Buffer> m_Constants, m_Speed, m_SpeedReadback;
			ComPtr<ID3D11UnorderedAccessView> m_SpeedUAV;
			ComPtr<ID3D11SamplerState> m_Sampler;
			Texture m_V[2], m_S[3], m_Curl, m_Pressure, m_Divergence, m_Solid, m_Wall;
			std::vector<Vector4> m_WallUpload;
			ComPtr<ID3D11Texture3D> m_ScalarReadback, m_VelocityReadback, m_PressureReadback;
			AerodynamicsSimulationSettings m_Settings;
			Parameters m_P = {};
			float m_MaxRate = 0;
			enum { CLookup, CResponse, CInitialize, CApply, CRelax, CouplingKernelCount };
			struct CouplingBuffer
			{
				ComPtr<ID3D11Buffer> Data;
				ComPtr<ID3D11ShaderResourceView> SRV;
				ComPtr<ID3D11UnorderedAccessView> UAV;
				UINT Bytes = 0;
			};
			struct CouplingParameters { int Size[4]; Vector4 Weight; int Counts[4]; Vector4 Relaxation; } m_CP = {};
			ComPtr<ID3D11ComputeShader> m_CouplingKernels[CouplingKernelCount];
			ComPtr<ID3D11Buffer> m_CouplingConstants, m_ResponseReadback;
			CouplingParameters m_PreviousCP = {};
			bool m_CouplingConstantsSet = false;
			CouplingBuffer m_Rows, m_RowEntries, m_Cells, m_CellEntries, m_Response, m_Faces, m_Lookup, m_ActiveCells;
			Texture m_PressureState, m_SurfacePressure, m_PreviousPressure, m_NextPressure, m_SurfaceWall;
			bool m_CouplingReady = false;

			bool Upload(CouplingBuffer& B, size_t Count, UINT Stride, const void* Data = nullptr)
			{
				const UINT Bytes = (UINT)std::max<size_t>(1, Count) * Stride;
				if (B.Bytes < Bytes)
				{
					B = CouplingBuffer(); D3D11_BUFFER_DESC D = {}; D.ByteWidth = Bytes;
					D.Usage = D3D11_USAGE_DEFAULT; D.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
					D.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED; D.StructureByteStride = Stride;
					if (FAILED(m_Device->CreateBuffer(&D, nullptr, &B.Data)) || FAILED(m_Device->CreateShaderResourceView(B.Data.Get(), nullptr, &B.SRV))
						|| FAILED(m_Device->CreateUnorderedAccessView(B.Data.Get(), nullptr, &B.UAV))) return false;
					B.Bytes = Bytes;
				}
				if (Data && Count)
				{
					D3D11_BOX Box = { 0, 0, 0, (UINT)Count * Stride, 1, 1 };
					m_Context->UpdateSubresource(B.Data.Get(), 0, &Box, Data, 0, 0);
				}
				return true;
			}
			bool InitializeCoupling()
			{
				if (m_CouplingReady) return true;
				const char* Names[] = { "BuildLookup", "SurfaceResponse", "InitializeState", "ApplyCorrection", "RelaxPressure" };
				for (int I = 0; I < CouplingKernelCount; ++I)
				{
					ComPtr<ID3DBlob> Code, Errors;
					if (FAILED(D3DCompile(PressureSource, std::strlen(PressureSource), "AerodynamicsPressureKernels", nullptr, nullptr,
						Names[I], "cs_5_0", D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &Code, &Errors)))
					{
						if (Errors) std::fprintf(stderr, "Pressure shader %s: %s\n", Names[I], (const char*)Errors->GetBufferPointer());
						return false;
					}
					if (FAILED(m_Device->CreateComputeShader(Code->GetBufferPointer(), Code->GetBufferSize(), nullptr, &m_CouplingKernels[I]))) return false;
				}
				D3D11_BUFFER_DESC D = {}; D.ByteWidth = sizeof(CouplingParameters); D.Usage = D3D11_USAGE_DEFAULT; D.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
				if (FAILED(m_Device->CreateBuffer(&D, nullptr, &m_CouplingConstants))) return false;
				if (!CreateTexture(m_PressureState, DXGI_FORMAT_R32G32B32A32_FLOAT)) return false;
				if (!CreateTexture(m_SurfacePressure, DXGI_FORMAT_R32_FLOAT) || !CreateTexture(m_SurfaceWall, DXGI_FORMAT_R32G32B32A32_FLOAT)) return false;
				if (!CreateTexture(m_PreviousPressure, DXGI_FORMAT_R32_FLOAT) || !CreateTexture(m_NextPressure, DXGI_FORMAT_R32_FLOAT)) return false;
				m_CouplingReady = true; return true;
			}
			void DispatchCoupling(int Kernel, UINT Groups)
			{
				if (!m_CouplingConstantsSet || std::memcmp(&m_CP, &m_PreviousCP, sizeof(m_CP)) != 0)
				{
					m_Context->UpdateSubresource(m_CouplingConstants.Get(), 0, nullptr, &m_CP, 0, 0);
					m_PreviousCP = m_CP; m_CouplingConstantsSet = true;
				}
				ID3D11Buffer* CB = m_CouplingConstants.Get(); m_Context->CSSetConstantBuffers(0, 1, &CB);
				ID3D11ShaderResourceView* SRVs[] = { m_PressureState.SRV.Get(), m_SurfacePressure.SRV.Get(), m_PreviousPressure.SRV.Get(), m_V[0].SRV.Get(), m_Wall.SRV.Get(), m_Solid.SRV.Get(),
					m_Rows.SRV.Get(), m_RowEntries.SRV.Get(), m_Cells.SRV.Get(), m_CellEntries.SRV.Get(), m_Response.SRV.Get(), m_Faces.SRV.Get(), m_Lookup.SRV.Get(), m_ActiveCells.SRV.Get() };
				ID3D11UnorderedAccessView* UAVs[8] = {};
				if (Kernel == CInitialize) { UAVs[0] = m_PressureState.UAV.Get(); SRVs[0] = nullptr; }
				if (Kernel == CApply) UAVs[1] = m_Pressure.UAV.Get();
				if (Kernel == CRelax) UAVs[1] = m_NextPressure.UAV.Get();
				if (Kernel == CResponse) { UAVs[2] = m_Response.UAV.Get(); SRVs[10] = nullptr; }
				if (Kernel == CLookup) { UAVs[3] = m_Cells.UAV.Get(); SRVs[8] = nullptr; UAVs[5] = m_Lookup.UAV.Get(); SRVs[12] = nullptr; }
				if (Kernel == CApply) { UAVs[6] = m_V[1].UAV.Get(); UAVs[7] = m_SurfaceWall.UAV.Get(); }
				m_Context->CSSetShaderResources(0, 14, SRVs); m_Context->CSSetUnorderedAccessViews(0, 8, UAVs, nullptr);
				m_Context->CSSetShader(m_CouplingKernels[Kernel].Get(), nullptr, 0);
				if (Kernel == CInitialize || Kernel == CRelax || Kernel == CApply) m_Context->Dispatch((m_P.Size[0] + 7) / 8, (m_P.Size[1] + 3) / 4, (m_P.Size[2] + 3) / 4);
				else m_Context->Dispatch(std::max(1u, Groups), 1, 1);
				std::fill(SRVs, SRVs + 14, nullptr); std::fill(UAVs, UAVs + 8, nullptr);
				m_Context->CSSetShaderResources(0, 14, SRVs); m_Context->CSSetUnorderedAccessViews(0, 8, UAVs, nullptr);
				if (Kernel == CRelax) { std::swap(m_PreviousPressure, m_SurfacePressure); std::swap(m_SurfacePressure, m_NextPressure); }
			}

			bool CreateTexture(Texture& T, DXGI_FORMAT Format)
			{
				D3D11_TEXTURE3D_DESC Desc = {};
				Desc.Width = m_P.Size[0]; Desc.Height = m_P.Size[1]; Desc.Depth = m_P.Size[2];
				Desc.MipLevels = 1; Desc.Format = Format; Desc.Usage = D3D11_USAGE_DEFAULT;
				Desc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
				return SUCCEEDED(m_Device->CreateTexture3D(&Desc, nullptr, &T.Data))
					&& SUCCEEDED(m_Device->CreateShaderResourceView(T.Data.Get(), nullptr, &T.SRV))
					&& SUCCEEDED(m_Device->CreateUnorderedAccessView(T.Data.Get(), nullptr, &T.UAV));
			}

			void Dispatch(int Kernel, Texture* VOut = nullptr, Texture* SOut = nullptr)
			{
				m_Context->UpdateSubresource(m_Constants.Get(), 0, nullptr, &m_P, 0, 0);
				ID3D11Buffer* CB = m_Constants.Get(); m_Context->CSSetConstantBuffers(0, 1, &CB);
				ID3D11SamplerState* Sampler = m_Sampler.Get(); m_Context->CSSetSamplers(0, 1, &Sampler);
				ID3D11ShaderResourceView* SRVs[8] = { m_V[0].SRV.Get(), m_S[0].SRV.Get(), m_S[1].SRV.Get(),
					m_Curl.SRV.Get(), m_Pressure.SRV.Get(), m_Solid.SRV.Get(), m_Divergence.SRV.Get(), m_Wall.SRV.Get() };
				if (Kernel == Curl) SRVs[3] = nullptr;
				if (Kernel == Pressure) SRVs[4] = nullptr;
				if (Kernel == Divergence) SRVs[6] = nullptr;
				if (SOut == &m_S[1]) SRVs[2] = nullptr;
				ID3D11UnorderedAccessView* UAVs[5] = { VOut ? VOut->UAV.Get() : nullptr, SOut ? SOut->UAV.Get() : nullptr,
					Kernel == Pressure ? m_Pressure.UAV.Get() : nullptr,
					Kernel == Divergence ? m_Divergence.UAV.Get() : nullptr, Kernel == MaxRate ? m_SpeedUAV.Get() : nullptr };
				m_Context->CSSetShaderResources(0, 8, SRVs);
				m_Context->CSSetUnorderedAccessViews(0, 5, UAVs, nullptr);
				m_Context->CSSetShader(m_Kernels[Kernel].Get(), nullptr, 0);
				m_Context->Dispatch((m_P.Size[0] + 3) / 4, (m_P.Size[1] + 3) / 4, (m_P.Size[2] + 3) / 4);
				std::fill(SRVs, SRVs + 8, nullptr); std::fill(UAVs, UAVs + 5, nullptr);
				m_Context->CSSetShaderResources(0, 8, SRVs);
				m_Context->CSSetUnorderedAccessViews(0, 5, UAVs, nullptr);
			}

		public:
			bool ProjectSurfaceMass(const AerodynamicsPressureSystem& S, std::vector<float>& Response) override
			{
				if (!InitializeCoupling()) return false;
				const int Rows = (int)S.RowOffsets.size() - 1, Count = m_P.Size[0] * m_P.Size[1] * m_P.Size[2], Groups = (Count + 511) / 512;
				std::vector<Vector3i> ActiveCells; ActiveCells.reserve(S.CellIndices.size());
				for (size_t I = 0; I < S.CellIndices.size(); ++I) ActiveCells.emplace_back(S.CellIndices[I], S.CellOffsets[I], S.CellOffsets[I + 1]);
				if (!Upload(m_Rows, S.RowOffsets.size(), sizeof(int), S.RowOffsets.data())
					|| !Upload(m_RowEntries, S.RowEntries.size(), sizeof(AerodynamicsPressureEntry), S.RowEntries.data())
					|| !Upload(m_Cells, Count, 2 * sizeof(int))
					|| !Upload(m_ActiveCells, ActiveCells.size(), sizeof(Vector3i), ActiveCells.data())
					|| !Upload(m_CellEntries, S.CellEntries.size(), sizeof(AerodynamicsPressureEntry), S.CellEntries.data())
					|| !Upload(m_Response, Rows, sizeof(float)) || !Upload(m_Faces, S.Faces.size(), sizeof(AerodynamicsPressureFace), S.Faces.data())
					|| !Upload(m_Lookup, Count * 3, sizeof(unsigned))) return false;
				for (int A = 0; A < 3; ++A) { m_CP.Size[A] = m_P.Size[A]; m_CP.Weight[A] = 1 / (m_P.CellDt[A] * m_P.CellDt[A]); }
				m_CP.Size[3] = Rows; m_CP.Counts[0] = (int)S.Faces.size(); m_CP.Counts[1] = (int)ActiveCells.size(); m_CP.Counts[2] = 0;
				const UINT Empty[] = { UINT(-1), UINT(-1), UINT(-1), UINT(-1) }; m_Context->ClearUnorderedAccessViewUint(m_Lookup.UAV.Get(), Empty);
				const UINT EmptyCells[4] = {}; m_Context->ClearUnorderedAccessViewUint(m_Cells.UAV.Get(), EmptyCells);
				DispatchCoupling(CLookup, ((UINT)std::max(S.Faces.size(), ActiveCells.size()) + 511) / 512);
				const float Zero[4] = {};
				m_Context->ClearUnorderedAccessViewFloat(m_SurfacePressure.UAV.Get(), Zero);
				m_Context->ClearUnorderedAccessViewFloat(m_PreviousPressure.UAV.Get(), Zero);
				DispatchCoupling(CInitialize, Groups);
				// The absolute row-sum bound preconditions the full A + K^T K operator
				// into [0, 2]. Chebyshev acceleration needs no global dot-product stalls.
				float Alpha = 0; constexpr float Theta = 1.001f, Delta = 0.999f;
				for (int I = 0; I < std::max(40, m_Settings.PressureIterations); ++I)
				{
					const float Beta = I == 0 ? 0 : (I == 1 ? 0.5f : 0.25f) * Delta * Delta * Alpha * Alpha;
					const float PreviousAlpha = Alpha;
					Alpha = 1 / (Theta - (I == 0 ? 0 : Beta / Alpha));
					m_CP.Relaxation = Vector4(Alpha, I == 0 ? 0 : Beta * Alpha / PreviousAlpha, 0, 0); m_CP.Counts[2] = 3;
					DispatchCoupling(CResponse, (Rows + 511) / 512); DispatchCoupling(CRelax, Groups);
				}
				// Dampen the high-frequency error remaining after the Chebyshev sweep.
				m_CP.Relaxation = Vector4(2.0f / 3, 0, 0, 0);
				for (int I = 0; I < 4; ++I) { DispatchCoupling(CResponse, (Rows + 511) / 512); DispatchCoupling(CRelax, Groups); }
				m_CP.Counts[2] = 2; DispatchCoupling(CResponse, (Rows + 511) / 512); DispatchCoupling(CApply, Groups);
				std::swap(m_Wall, m_SurfaceWall); std::swap(m_V[0], m_V[1]);
				D3D11_BUFFER_DESC D = {}; if (m_ResponseReadback) m_ResponseReadback->GetDesc(&D);
				if (D.ByteWidth != m_Response.Bytes)
				{
					m_ResponseReadback.Reset(); D = {}; D.ByteWidth = m_Response.Bytes; D.Usage = D3D11_USAGE_STAGING; D.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
					if (FAILED(m_Device->CreateBuffer(&D, nullptr, &m_ResponseReadback))) return false;
				}
				m_Context->CopyResource(m_ResponseReadback.Get(), m_Response.Data.Get()); D3D11_MAPPED_SUBRESOURCE Map;
				if (FAILED(m_Context->Map(m_ResponseReadback.Get(), 0, D3D11_MAP_READ, 0, &Map))) return false;
				Response.resize(Rows); std::memcpy(Response.data(), Map.pData, Rows * sizeof(float)); m_Context->Unmap(m_ResponseReadback.Get(), 0);
				for (float R : Response) if (!std::isfinite(R)) return false;
				return SUCCEEDED(m_Device->GetDeviceRemovedReason());
			}
			bool Initialize(const AerodynamicsSimulationSettings& Settings, const std::vector<unsigned char>& Solid)
			{
				if (Settings.Emitters.size() > 8) return false;
				m_Settings = Settings;
				D3D_FEATURE_LEVEL Level;
				if (FAILED(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0,
					D3D11_SDK_VERSION, &m_Device, &Level, &m_Context)) || Level < D3D_FEATURE_LEVEL_11_0) return false;
				const char* Entries[] = { "AdvectVelocity", "Emit", "Curl", "ApplyForces", "Divergence", "Pressure", "Project",
					"AdvectScalar", "CorrectScalar", "Impulse", "MaxRate", "Boundary" };
				for (int I = 0; I < KernelCount; ++I)
				{
					ComPtr<ID3DBlob> Code, Errors;
					if (FAILED(D3DCompile(AerodynamicsSource, std::strlen(AerodynamicsSource), "AerodynamicsComputeKernels", nullptr, nullptr,
						Entries[I], "cs_5_0", D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &Code, &Errors)))
					{
						if (Errors) std::fprintf(stderr, "Aerodynamics shader %s: %s\n", Entries[I], (const char*)Errors->GetBufferPointer());
						return false;
					}
					if (FAILED(m_Device->CreateComputeShader(Code->GetBufferPointer(), Code->GetBufferSize(), nullptr, &m_Kernels[I]))) return false;
				}
				for (int A = 0; A < 3; ++A) m_P.Size[A] = Settings.Resolution[A];
				Vector3 H = Settings.BoundsMax - Settings.BoundsMin;
				m_P.CellDt = Vector4(H.x / Settings.Resolution.x, H.y / Settings.Resolution.y, H.z / Settings.Resolution.z, 0);
				m_P.MinimumTime = Vector4(Settings.BoundsMin.x, Settings.BoundsMin.y, Settings.BoundsMin.z, 0);
				m_P.Forces = Vector4(Settings.Buoyancy, Settings.TracerWeight, Settings.Vorticity, 0);
				const int ShortAxis = std::min(Settings.Resolution.x, std::min(Settings.Resolution.y, Settings.Resolution.z));
				m_P.Forces.w = std::max(1.75f, std::min(1.93f, 2.0f / (1.0f + std::sin(3.14159265f / ShortAxis))));
				m_P.Decay = Vector4(Settings.DensityDissipation, Settings.Cooling, (float)Settings.Emitters.size(), 0);
				for (size_t I = 0; I < Settings.Emitters.size(); ++I)
				{
					const AerodynamicsEmitter& E = Settings.Emitters[I];
					m_P.EmitterPosition[I] = Vector4(E.Position.x, E.Position.y, E.Position.z, E.Radius);
					m_P.EmitterVelocity[I] = Vector4(E.Velocity.x, E.Velocity.y, E.Velocity.z, 0);
					m_P.EmitterScalar[I] = Vector4(E.Density, E.Temperature, 0, 0);
				}
				for (auto& V : m_V) if (!CreateTexture(V, DXGI_FORMAT_R32G32B32A32_FLOAT)) return false;
				for (auto& S : m_S) if (!CreateTexture(S, DXGI_FORMAT_R32G32_FLOAT)) return false;
				if (!CreateTexture(m_Curl, DXGI_FORMAT_R32G32B32A32_FLOAT)
					|| !CreateTexture(m_Pressure, DXGI_FORMAT_R32_FLOAT) || !CreateTexture(m_Divergence, DXGI_FORMAT_R32_FLOAT)
					|| !CreateTexture(m_Solid, DXGI_FORMAT_R8_UINT) || !CreateTexture(m_Wall, DXGI_FORMAT_R32G32B32A32_FLOAT)) return false;
				const float Zero[4] = {}; m_Context->ClearUnorderedAccessViewFloat(m_Wall.UAV.Get(), Zero);
				m_Context->UpdateSubresource(m_Solid.Data.Get(), 0, nullptr, Solid.data(), m_P.Size[0], m_P.Size[0] * m_P.Size[1]);
				D3D11_TEXTURE3D_DESC Desc;
				m_S[0].Data->GetDesc(&Desc); Desc.Usage = D3D11_USAGE_STAGING; Desc.BindFlags = 0; Desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
				if (FAILED(m_Device->CreateTexture3D(&Desc, nullptr, &m_ScalarReadback))) return false;
				Desc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
				if (FAILED(m_Device->CreateTexture3D(&Desc, nullptr, &m_VelocityReadback))) return false;
				Desc.Format = DXGI_FORMAT_R32_FLOAT;
				if (FAILED(m_Device->CreateTexture3D(&Desc, nullptr, &m_PressureReadback))) return false;
				D3D11_BUFFER_DESC Buffer = {}; Buffer.ByteWidth = sizeof(Parameters); Buffer.Usage = D3D11_USAGE_DEFAULT;
				Buffer.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
				if (FAILED(m_Device->CreateBuffer(&Buffer, nullptr, &m_Constants))) return false;
				Buffer.ByteWidth = 4; Buffer.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
				if (FAILED(m_Device->CreateBuffer(&Buffer, nullptr, &m_Speed))) return false;
				D3D11_UNORDERED_ACCESS_VIEW_DESC UAV = {}; UAV.Format = DXGI_FORMAT_R32_UINT;
				UAV.ViewDimension = D3D11_UAV_DIMENSION_BUFFER; UAV.Buffer.NumElements = 1;
				if (FAILED(m_Device->CreateUnorderedAccessView(m_Speed.Get(), &UAV, &m_SpeedUAV))) return false;
				Buffer.BindFlags = 0; Buffer.Usage = D3D11_USAGE_STAGING; Buffer.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
				if (FAILED(m_Device->CreateBuffer(&Buffer, nullptr, &m_SpeedReadback))) return false;
				D3D11_SAMPLER_DESC Sampler = {}; Sampler.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
				Sampler.AddressU = Sampler.AddressV = Sampler.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
				Sampler.MaxLOD = D3D11_FLOAT32_MAX;
				if (FAILED(m_Device->CreateSamplerState(&Sampler, &m_Sampler))) return false;
				Reset(); return true;
			}

			bool UpdateObstacles(const std::vector<unsigned char>& Solid, const std::vector<Vector3>& Velocity, const AerodynamicsGridRegion* Region) override
			{
				m_WallUpload.resize(Velocity.size());
				const Vector3i Lo = Region ? Region->Min : Vector3i(0), Hi = Region ? Region->Max : Vector3i(m_P.Size[0], m_P.Size[1], m_P.Size[2]);
				if (Lo.x >= Hi.x || Lo.y >= Hi.y || Lo.z >= Hi.z) return true;
				for (int Z = Lo.z; Z < Hi.z; ++Z) for (int Y = Lo.y; Y < Hi.y; ++Y) for (int X = Lo.x; X < Hi.x; ++X)
				{
					const int I = X + m_P.Size[0] * (Y + m_P.Size[1] * Z);
					const Vector3& V = Velocity[I];
					m_WallUpload[I] = Vector4(V.x, V.y, V.z, 0);
					for (int A = 0; A < 3; ++A) m_MaxRate = std::max(m_MaxRate, std::fabs(V[A]) / m_P.CellDt[A]);
				}
				const int Start = Lo.x + m_P.Size[0] * (Lo.y + m_P.Size[1] * Lo.z);
				const D3D11_BOX Box = { (UINT)Lo.x, (UINT)Lo.y, (UINT)Lo.z, (UINT)Hi.x, (UINT)Hi.y, (UINT)Hi.z };
				m_Context->UpdateSubresource(m_Solid.Data.Get(), 0, &Box, Solid.data() + Start, m_P.Size[0], m_P.Size[0] * m_P.Size[1]);
				m_Context->UpdateSubresource(m_Wall.Data.Get(), 0, &Box, m_WallUpload.data() + Start, m_P.Size[0] * sizeof(Vector4), m_P.Size[0] * m_P.Size[1] * sizeof(Vector4));
				Dispatch(Boundary, &m_V[1], &m_S[2]); std::swap(m_V[0], m_V[1]); std::swap(m_S[0], m_S[2]);
				return SUCCEEDED(m_Device->GetDeviceRemovedReason());
			}

			void Reset() override
			{
				const float Zero[4] = {};
				for (auto& V : m_V) m_Context->ClearUnorderedAccessViewFloat(V.UAV.Get(), Zero);
				for (auto& S : m_S) m_Context->ClearUnorderedAccessViewFloat(S.UAV.Get(), Zero);
				m_MaxRate = 0;
			}

			void Step(float Dt, float Time, bool Transport, bool ProjectPressure) override
			{
				m_P.CellDt.w = Dt; m_P.MinimumTime.w = Time;
				Dispatch(AdvectV, &m_V[1]); std::swap(m_V[0], m_V[1]);
				Dispatch(EmitSource, &m_V[1], &m_S[2]); std::swap(m_V[0], m_V[1]); std::swap(m_S[0], m_S[2]);
				Dispatch(Curl, &m_Curl);
				Dispatch(Force, &m_V[1]); std::swap(m_V[0], m_V[1]);
				Dispatch(Divergence);
				const float Zero[4] = {}; m_Context->ClearUnorderedAccessViewFloat(m_Pressure.UAV.Get(), Zero);
				if (ProjectPressure)
				{
					for (int I = 0; I < m_Settings.PressureIterations; ++I)
					for (int Parity = 0; Parity < 2; ++Parity) { m_P.Size[3] = Parity; Dispatch(Pressure); }
					Dispatch(Project, &m_V[1]); std::swap(m_V[0], m_V[1]);
				}
				if (Transport) TransportScalars();
			}
			void TransportScalars() override
			{
				Dispatch(AdvectS, nullptr, &m_S[1]);
				Dispatch(CorrectS, nullptr, &m_S[2]); std::swap(m_S[0], m_S[2]);
			}

			void ApplyImpulse(const Vector3& Position, float Radius, const Vector3& Change) override
			{
				m_P.ImpulsePosition = Vector4(Position.x, Position.y, Position.z, Radius);
				m_P.ImpulseChange = Vector4(Change.x, Change.y, Change.z, 0);
				Dispatch(Impulse, &m_V[1]); std::swap(m_V[0], m_V[1]);
				m_MaxRate += Change.Length() / std::min(m_P.CellDt.x, std::min(m_P.CellDt.y, m_P.CellDt.z));
			}

			bool Readback(std::vector<float>& Density, std::vector<float>& Temperature) override
			{
				const UINT Zero[4] = {}; m_Context->ClearUnorderedAccessViewUint(m_SpeedUAV.Get(), Zero);
				Dispatch(MaxRate);
				m_Context->CopyResource(m_SpeedReadback.Get(), m_Speed.Get());
				m_Context->CopyResource(m_ScalarReadback.Get(), m_S[0].Data.Get());
				D3D11_MAPPED_SUBRESOURCE Map;
				if (FAILED(m_Context->Map(m_ScalarReadback.Get(), 0, D3D11_MAP_READ, 0, &Map))) return false;
				for (int Z = 0; Z < m_P.Size[2]; ++Z)
				for (int Y = 0; Y < m_P.Size[1]; ++Y)
				{
					const float* Row = (const float*)((const char*)Map.pData + Z * Map.DepthPitch + Y * Map.RowPitch);
					const int Base = m_P.Size[0] * (Y + m_P.Size[1] * Z);
					for (int X = 0; X < m_P.Size[0]; ++X) { Density[Base + X] = Row[X * 2]; Temperature[Base + X] = Row[X * 2 + 1]; }
				}
				m_Context->Unmap(m_ScalarReadback.Get(), 0);
				if (FAILED(m_Context->Map(m_SpeedReadback.Get(), 0, D3D11_MAP_READ, 0, &Map))) return false;
				std::memcpy(&m_MaxRate, Map.pData, sizeof(float)); m_Context->Unmap(m_SpeedReadback.Get(), 0);
				return true;
			}

			bool ReadVelocity(std::vector<Vector3>& Velocity) const override
			{
				m_Context->CopyResource(m_VelocityReadback.Get(), m_V[0].Data.Get());
				D3D11_MAPPED_SUBRESOURCE Map;
				if (FAILED(m_Context->Map(m_VelocityReadback.Get(), 0, D3D11_MAP_READ, 0, &Map))) return false;
				for (int Z = 0; Z < m_P.Size[2]; ++Z)
				for (int Y = 0; Y < m_P.Size[1]; ++Y)
				{
					const float* Row = (const float*)((const char*)Map.pData + Z * Map.DepthPitch + Y * Map.RowPitch);
					const int Base = m_P.Size[0] * (Y + m_P.Size[1] * Z);
					for (int X = 0; X < m_P.Size[0]; ++X) Velocity[Base + X] = Vector3(Row[X * 4], Row[X * 4 + 1], Row[X * 4 + 2]);
				}
				m_Context->Unmap(m_VelocityReadback.Get(), 0); return true;
			}
			float GetMaxRate() const override { return m_MaxRate; }
			bool ReadPressure(std::vector<float>& Pressure) const override
			{
				m_Context->CopyResource(m_PressureReadback.Get(), m_Pressure.Data.Get());
				D3D11_MAPPED_SUBRESOURCE Map;
				if (FAILED(m_Context->Map(m_PressureReadback.Get(), 0, D3D11_MAP_READ, 0, &Map))) return false;
				for (int Z = 0; Z < m_P.Size[2]; ++Z)
				for (int Y = 0; Y < m_P.Size[1]; ++Y)
				{
					const char* Row = (const char*)Map.pData + Z * Map.DepthPitch + Y * Map.RowPitch;
					std::memcpy(Pressure.data() + m_P.Size[0] * (Y + m_P.Size[1] * Z), Row, m_P.Size[0] * sizeof(float));
				}
				m_Context->Unmap(m_PressureReadback.Get(), 0); return true;
			}
		};
	}

	std::unique_ptr<AerodynamicsGPUBackend> AerodynamicsGPUBackend::CreateComputeShader(const AerodynamicsSimulationSettings& Settings,
		const std::vector<unsigned char>& Solid)
	{
		std::unique_ptr<ComputeAerodynamics> Backend(new ComputeAerodynamics());
		if (!Backend->Initialize(Settings, Solid)) return nullptr;
		return std::move(Backend);
	}
}
#else
namespace Riemann
{
	std::unique_ptr<AerodynamicsGPUBackend> AerodynamicsGPUBackend::CreateComputeShader(const AerodynamicsSimulationSettings&,
		const std::vector<unsigned char>&) { return nullptr; }
}
#endif
