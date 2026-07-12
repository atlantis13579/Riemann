#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <new>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <d3d11.h>
#include <d3dcompiler.h>
// Keep library directives for consumers that use the legacy Visual Studio projects.
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "d3dcompiler.lib")
#undef min
#undef max
#endif

namespace Riemann
{
#if defined(_WIN32)
	namespace
	{
		template <class T>
		void SafeRelease(T*& Object)
		{
			if (Object)
			{
				Object->Release();
				Object = nullptr;
			}
		}

		const char* ComputeShaderSource = R"HLSL(
ByteAddressBuffer InputData : register(t0);
RWByteAddressBuffer OutputData : register(u0);

cbuffer SolverConstants : register(b0)
{
	uint MatrixOffsetBytes;
	uint BiasOffsetBytes;
	uint LowerOffsetBytes;
	uint UpperOffsetBytes;
	uint FrictionCoefficientOffsetBytes;
	uint InitialLambdaOffsetBytes;
	uint FrictionNormalOffsetBytes;
	uint MatrixOffsetsOffsetBytes;
	uint VectorOffsetsOffsetBytes;
	uint DimensionsOffsetBytes;
	uint LambdaOutputOffsetBytes;
	uint ResultsOutputOffsetBytes;
	uint IslandBase;
	uint DispatchIslandCount;
	uint MaxIterations;
	float Tolerance;
	float Relaxation;
	uint Padding0;
	uint Padding1;
	uint Padding2;
};

groupshared float SharedReduction[64];
groupshared float SharedResidual;
groupshared uint SharedIterations;
groupshared uint SharedConverged;

float LoadFloat(uint Base, uint Index)
{
	return asfloat(InputData.Load(Base + Index * 4));
}

int LoadInt(uint Base, uint Index)
{
	return asint(InputData.Load(Base + Index * 4));
}

float LoadLambda(uint Index)
{
	return asfloat(OutputData.Load(LambdaOutputOffsetBytes + Index * 4));
}

void StoreLambda(uint Index, float Value)
{
	OutputData.Store(LambdaOutputOffsetBytes + Index * 4, asuint(Value));
}

void GetBounds(uint VectorOffset, uint Row, out float LowerBound, out float UpperBound)
{
	LowerBound = LoadFloat(LowerOffsetBytes, VectorOffset + Row);
	UpperBound = LoadFloat(UpperOffsetBytes, VectorOffset + Row);
	const int NormalRow = LoadInt(FrictionNormalOffsetBytes, VectorOffset + Row);
	if (NormalRow >= 0)
	{
		const float Limit = LoadFloat(FrictionCoefficientOffsetBytes, VectorOffset + Row)
			* max(0.0f, LoadLambda(VectorOffset + (uint)NormalRow));
		LowerBound = max(LowerBound, -Limit);
		UpperBound = min(UpperBound, Limit);
	}
}

[numthreads(64, 1, 1)]
void SolveProjectedGaussSeidel(uint3 GroupId : SV_GroupID,
	uint3 GroupThreadId : SV_GroupThreadID)
{
	const uint LocalIsland = GroupId.x;
	if (LocalIsland >= DispatchIslandCount)
	{
		return;
	}
	const uint Thread = GroupThreadId.x;
	const uint Island = IslandBase + LocalIsland;
	const uint Dimension = (uint)LoadInt(DimensionsOffsetBytes, Island);
	const uint MatrixOffset = (uint)LoadInt(MatrixOffsetsOffsetBytes, Island);
	const uint VectorOffset = (uint)LoadInt(VectorOffsetsOffsetBytes, Island);

	for (uint InitialRow = Thread; InitialRow < Dimension; InitialRow += 64)
	{
		StoreLambda(VectorOffset + InitialRow,
			LoadFloat(InitialLambdaOffsetBytes, VectorOffset + InitialRow));
	}
	AllMemoryBarrierWithGroupSync();

	if (Thread == 0)
	{
		SharedIterations = 0;
		SharedConverged = 0;
		SharedResidual = 0.0f;
		for (uint ClampRow = 0; ClampRow < Dimension; ++ClampRow)
		{
			float LowerBound;
			float UpperBound;
			GetBounds(VectorOffset, ClampRow, LowerBound, UpperBound);
			StoreLambda(VectorOffset + ClampRow,
				max(LowerBound, min(UpperBound, LoadLambda(VectorOffset + ClampRow))));
		}
	}
	AllMemoryBarrierWithGroupSync();

	// Shader model 5 requires every group-sync barrier to remain in statically
	// uniform control flow, so the fixed iteration budget cannot early-out here.
	// Convergence is still decided by the final projected residual below.
	for (uint Iteration = 0; Iteration < MaxIterations; ++Iteration)
	{
		for (uint SolveRow = 0; SolveRow < Dimension; ++SolveRow)
		{
			float Partial = 0.0f;
			for (uint Column = Thread; Column < Dimension; Column += 64)
			{
				Partial += LoadFloat(MatrixOffsetBytes,
					MatrixOffset + SolveRow * Dimension + Column)
					* LoadLambda(VectorOffset + Column);
			}
			SharedReduction[Thread] = Partial;
			GroupMemoryBarrierWithGroupSync();
			for (uint Stride = 32; Stride > 0; Stride >>= 1)
			{
				if (Thread < Stride)
				{
					SharedReduction[Thread] += SharedReduction[Thread + Stride];
				}
				GroupMemoryBarrierWithGroupSync();
			}

			if (Thread == 0)
			{
				float LowerBound;
				float UpperBound;
				GetBounds(VectorOffset, SolveRow, LowerBound, UpperBound);
				const float OldLambda = LoadLambda(VectorOffset + SolveRow);
				const float Diagonal = LoadFloat(MatrixOffsetBytes,
					MatrixOffset + SolveRow * Dimension + SolveRow);
				const float W = LoadFloat(BiasOffsetBytes, VectorOffset + SolveRow)
					+ SharedReduction[0];
				const float NewLambda = max(LowerBound, min(UpperBound,
					OldLambda - Relaxation * W / Diagonal));
				StoreLambda(VectorOffset + SolveRow, NewLambda);
			}
			AllMemoryBarrierWithGroupSync();
		}

		if (Thread == 0)
		{
			SharedIterations = Iteration + 1;
		}
		GroupMemoryBarrierWithGroupSync();
	}

	if (Thread == 0)
	{
		SharedResidual = 0.0f;
	}
	GroupMemoryBarrierWithGroupSync();
	for (uint ResidualRow = 0; ResidualRow < Dimension; ++ResidualRow)
	{
		float Partial = 0.0f;
		for (uint Column = Thread; Column < Dimension; Column += 64)
		{
			Partial += LoadFloat(MatrixOffsetBytes,
				MatrixOffset + ResidualRow * Dimension + Column)
				* LoadLambda(VectorOffset + Column);
		}
		SharedReduction[Thread] = Partial;
		GroupMemoryBarrierWithGroupSync();
		for (uint Stride = 32; Stride > 0; Stride >>= 1)
		{
			if (Thread < Stride)
			{
				SharedReduction[Thread] += SharedReduction[Thread + Stride];
			}
			GroupMemoryBarrierWithGroupSync();
		}
		if (Thread == 0)
		{
			float LowerBound;
			float UpperBound;
			GetBounds(VectorOffset, ResidualRow, LowerBound, UpperBound);
			const float Lambda = LoadLambda(VectorOffset + ResidualRow);
			const float Diagonal = LoadFloat(MatrixOffsetBytes,
				MatrixOffset + ResidualRow * Dimension + ResidualRow);
			const float W = LoadFloat(BiasOffsetBytes, VectorOffset + ResidualRow)
				+ SharedReduction[0];
			const float Projected = max(LowerBound, min(UpperBound,
				Lambda - W / Diagonal));
			SharedResidual = max(SharedResidual, abs(Projected - Lambda));
		}
		GroupMemoryBarrierWithGroupSync();
	}

	if (Thread == 0)
	{
		SharedConverged = SharedResidual <= Tolerance ? 1 : 0;
		const uint ResultAddress = ResultsOutputOffsetBytes + Island * 16;
		OutputData.Store(ResultAddress + 0, SharedIterations);
		OutputData.Store(ResultAddress + 4, asuint(SharedResidual));
		OutputData.Store(ResultAddress + 8, SharedConverged);
		OutputData.Store(ResultAddress + 12, 0);
	}
}
)HLSL";

		struct SolverConstants
		{
			uint32_t MatrixOffsetBytes;
			uint32_t BiasOffsetBytes;
			uint32_t LowerOffsetBytes;
			uint32_t UpperOffsetBytes;
			uint32_t FrictionCoefficientOffsetBytes;
			uint32_t InitialLambdaOffsetBytes;
			uint32_t FrictionNormalOffsetBytes;
			uint32_t MatrixOffsetsOffsetBytes;
			uint32_t VectorOffsetsOffsetBytes;
			uint32_t DimensionsOffsetBytes;
			uint32_t LambdaOutputOffsetBytes;
			uint32_t ResultsOutputOffsetBytes;
			uint32_t IslandBase;
			uint32_t DispatchIslandCount;
			uint32_t MaxIterations;
			float Tolerance;
			float Relaxation;
			uint32_t Padding[3];
		};

		static_assert(sizeof(SolverConstants) == 80,
			"Compute shader constant layout must match HLSL");

		struct InputLayout
		{
			size_t Matrix;
			size_t Bias;
			size_t Lower;
			size_t Upper;
			size_t FrictionCoefficient;
			size_t InitialLambda;
			size_t FrictionNormal;
			size_t MatrixOffsets;
			size_t VectorOffsets;
			size_t Dimensions;
			size_t TotalBytes;
		};

		struct OutputLayout
		{
			size_t Lambda;
			size_t Results;
			size_t TotalBytes;
		};

		bool CheckedAdd(size_t Left, size_t Right, size_t* Result)
		{
			if (Left > std::numeric_limits<size_t>::max() - Right)
			{
				return false;
			}
			*Result = Left + Right;
			return true;
		}

		bool CheckedMultiply(size_t Left, size_t Right, size_t* Result)
		{
			if (Left != 0 && Right > std::numeric_limits<size_t>::max() / Left)
			{
				return false;
			}
			*Result = Left * Right;
			return true;
		}

		bool AppendSection(size_t* Cursor, size_t ElementSize, size_t Count,
			size_t* Offset)
		{
			const size_t Aligned = (*Cursor + 15u) & ~size_t(15u);
			if (Aligned < *Cursor)
			{
				return false;
			}
			size_t Bytes = 0;
			if (!CheckedMultiply(ElementSize, Count, &Bytes)
				|| !CheckedAdd(Aligned, Bytes, Cursor))
			{
				return false;
			}
			*Offset = Aligned;
			return true;
		}

		bool BuildLayouts(size_t MatrixElementCount, size_t VectorElementCount,
			size_t IslandCount, InputLayout* Input, OutputLayout* Output)
		{
			size_t Cursor = 0;
			if (!AppendSection(&Cursor, sizeof(float), MatrixElementCount, &Input->Matrix)
				|| !AppendSection(&Cursor, sizeof(float), VectorElementCount, &Input->Bias)
				|| !AppendSection(&Cursor, sizeof(float), VectorElementCount, &Input->Lower)
				|| !AppendSection(&Cursor, sizeof(float), VectorElementCount, &Input->Upper)
				|| !AppendSection(&Cursor, sizeof(float), VectorElementCount,
					&Input->FrictionCoefficient)
				|| !AppendSection(&Cursor, sizeof(float), VectorElementCount,
					&Input->InitialLambda)
				|| !AppendSection(&Cursor, sizeof(int), VectorElementCount,
					&Input->FrictionNormal)
				|| !AppendSection(&Cursor, sizeof(int), IslandCount, &Input->MatrixOffsets)
				|| !AppendSection(&Cursor, sizeof(int), IslandCount, &Input->VectorOffsets)
				|| !AppendSection(&Cursor, sizeof(int), IslandCount, &Input->Dimensions))
			{
				return false;
			}
			Input->TotalBytes = (Cursor + 15u) & ~size_t(15u);
			if (Input->TotalBytes < Cursor)
			{
				return false;
			}

			Cursor = 0;
			if (!AppendSection(&Cursor, sizeof(float), VectorElementCount, &Output->Lambda)
				|| !AppendSection(&Cursor, 16u, IslandCount, &Output->Results))
			{
				return false;
			}
			Output->TotalBytes = (Cursor + 15u) & ~size_t(15u);
			return Output->TotalBytes >= Cursor
				&& Input->TotalBytes <= std::numeric_limits<UINT>::max()
				&& Output->TotalBytes <= std::numeric_limits<UINT>::max();
		}

		bool IsFiniteArray(const float* Values, size_t Count)
		{
			for (size_t Index = 0; Index < Count; ++Index)
			{
				if (!std::isfinite(Values[Index]))
				{
					return false;
				}
			}
			return true;
		}
	}

	struct LCPComputeShaderBatchContext
	{
		LCPComputeShaderBatchContext()
			: Device(nullptr)
			, DeviceContext(nullptr)
			, ComputeShader(nullptr)
			, ConstantBuffer(nullptr)
			, InputBuffer(nullptr)
			, InputView(nullptr)
			, InputCapacity(0)
			, OutputBuffer(nullptr)
			, OutputView(nullptr)
			, ReadbackBuffer(nullptr)
			, OutputCapacity(0)
		{
		}

		ID3D11Device* Device;
		ID3D11DeviceContext* DeviceContext;
		ID3D11ComputeShader* ComputeShader;
		ID3D11Buffer* ConstantBuffer;
		ID3D11Buffer* InputBuffer;
		ID3D11ShaderResourceView* InputView;
		size_t InputCapacity;
		ID3D11Buffer* OutputBuffer;
		ID3D11UnorderedAccessView* OutputView;
		ID3D11Buffer* ReadbackBuffer;
		size_t OutputCapacity;
		std::vector<unsigned char> HostInput;
		std::vector<unsigned char> HostOutput;
	};

	namespace
	{
		void ReleaseContextResources(LCPComputeShaderBatchContext* Context)
		{
			if (!Context)
			{
				return;
			}
			SafeRelease(Context->ReadbackBuffer);
			SafeRelease(Context->OutputView);
			SafeRelease(Context->OutputBuffer);
			SafeRelease(Context->InputView);
			SafeRelease(Context->InputBuffer);
			SafeRelease(Context->ConstantBuffer);
			SafeRelease(Context->ComputeShader);
			SafeRelease(Context->DeviceContext);
			SafeRelease(Context->Device);
			Context->InputCapacity = 0;
			Context->OutputCapacity = 0;
		}

		bool CreateHeadlessDevice(LCPComputeShaderBatchContext* Context)
		{
			const D3D_FEATURE_LEVEL RequestedLevels[] =
			{
				D3D_FEATURE_LEVEL_11_1,
				D3D_FEATURE_LEVEL_11_0,
			};
			D3D_FEATURE_LEVEL CreatedLevel = D3D_FEATURE_LEVEL_9_1;
			HRESULT Result = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE,
				nullptr, 0, RequestedLevels, 2, D3D11_SDK_VERSION,
				&Context->Device, &CreatedLevel, &Context->DeviceContext);
			if (Result == E_INVALIDARG)
			{
				Result = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE,
					nullptr, 0, &RequestedLevels[1], 1, D3D11_SDK_VERSION,
					&Context->Device, &CreatedLevel, &Context->DeviceContext);
			}
			return SUCCEEDED(Result) && Context->Device && Context->DeviceContext
				&& CreatedLevel >= D3D_FEATURE_LEVEL_11_0;
		}

		bool CreateShaderAndConstants(LCPComputeShaderBatchContext* Context)
		{
			UINT Flags = D3DCOMPILE_ENABLE_STRICTNESS;
#if defined(_DEBUG)
			Flags |= D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#else
			Flags |= D3DCOMPILE_OPTIMIZATION_LEVEL3;
#endif
			ID3DBlob* ShaderBlob = nullptr;
			ID3DBlob* ErrorBlob = nullptr;
			HRESULT Result = D3DCompile(ComputeShaderSource,
				std::strlen(ComputeShaderSource), "LCPSolverComputeShader", nullptr,
				nullptr, "SolveProjectedGaussSeidel", "cs_5_0", Flags, 0,
				&ShaderBlob, &ErrorBlob);
			if (ErrorBlob)
			{
#if defined(_DEBUG)
				OutputDebugStringA(static_cast<const char*>(ErrorBlob->GetBufferPointer()));
#endif
				ErrorBlob->Release();
			}
			if (FAILED(Result) || !ShaderBlob)
			{
				SafeRelease(ShaderBlob);
				return false;
			}
			Result = Context->Device->CreateComputeShader(ShaderBlob->GetBufferPointer(),
				ShaderBlob->GetBufferSize(), nullptr, &Context->ComputeShader);
			ShaderBlob->Release();
			if (FAILED(Result))
			{
				return false;
			}

			D3D11_BUFFER_DESC Desc = {};
			Desc.ByteWidth = sizeof(SolverConstants);
			Desc.Usage = D3D11_USAGE_DYNAMIC;
			Desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
			Desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
			return SUCCEEDED(Context->Device->CreateBuffer(
				&Desc, nullptr, &Context->ConstantBuffer));
		}

		size_t GrowCapacity(size_t Current, size_t Required)
		{
			size_t Capacity = std::max<size_t>(Current, 256u);
			while (Capacity < Required)
			{
				const size_t Growth = std::max<size_t>(Capacity / 2u, 256u);
				if (Capacity > std::numeric_limits<UINT>::max() - Growth)
				{
					Capacity = Required;
					break;
				}
				Capacity += Growth;
			}
			Capacity = (Capacity + 15u) & ~size_t(15u);
			return Capacity >= Required && Capacity <= std::numeric_limits<UINT>::max()
				? Capacity : 0;
		}

		bool EnsureInputCapacity(LCPComputeShaderBatchContext* Context, size_t Required)
		{
			if (Context->InputBuffer && Context->InputView
				&& Context->InputCapacity >= Required)
			{
				return true;
			}
			const size_t Capacity = GrowCapacity(Context->InputCapacity, Required);
			if (Capacity == 0)
			{
				return false;
			}

			ID3D11Buffer* Buffer = nullptr;
			ID3D11ShaderResourceView* View = nullptr;
			D3D11_BUFFER_DESC Desc = {};
			Desc.ByteWidth = static_cast<UINT>(Capacity);
			Desc.Usage = D3D11_USAGE_DEFAULT;
			Desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
			Desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
			if (FAILED(Context->Device->CreateBuffer(&Desc, nullptr, &Buffer)))
			{
				return false;
			}
			D3D11_SHADER_RESOURCE_VIEW_DESC ViewDesc = {};
			ViewDesc.Format = DXGI_FORMAT_R32_TYPELESS;
			ViewDesc.ViewDimension = D3D11_SRV_DIMENSION_BUFFEREX;
			ViewDesc.BufferEx.NumElements = static_cast<UINT>(Capacity / 4u);
			ViewDesc.BufferEx.Flags = D3D11_BUFFEREX_SRV_FLAG_RAW;
			if (FAILED(Context->Device->CreateShaderResourceView(Buffer, &ViewDesc, &View)))
			{
				Buffer->Release();
				return false;
			}

			SafeRelease(Context->InputView);
			SafeRelease(Context->InputBuffer);
			Context->InputBuffer = Buffer;
			Context->InputView = View;
			Context->InputCapacity = Capacity;
			return true;
		}

		bool EnsureOutputCapacity(LCPComputeShaderBatchContext* Context, size_t Required)
		{
			if (Context->OutputBuffer && Context->OutputView && Context->ReadbackBuffer
				&& Context->OutputCapacity >= Required)
			{
				return true;
			}
			const size_t Capacity = GrowCapacity(Context->OutputCapacity, Required);
			if (Capacity == 0)
			{
				return false;
			}

			ID3D11Buffer* Buffer = nullptr;
			ID3D11UnorderedAccessView* View = nullptr;
			ID3D11Buffer* Readback = nullptr;
			D3D11_BUFFER_DESC Desc = {};
			Desc.ByteWidth = static_cast<UINT>(Capacity);
			Desc.Usage = D3D11_USAGE_DEFAULT;
			Desc.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
			Desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
			if (FAILED(Context->Device->CreateBuffer(&Desc, nullptr, &Buffer)))
			{
				return false;
			}
			D3D11_UNORDERED_ACCESS_VIEW_DESC ViewDesc = {};
			ViewDesc.Format = DXGI_FORMAT_R32_TYPELESS;
			ViewDesc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
			ViewDesc.Buffer.NumElements = static_cast<UINT>(Capacity / 4u);
			ViewDesc.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
			if (FAILED(Context->Device->CreateUnorderedAccessView(Buffer, &ViewDesc, &View)))
			{
				Buffer->Release();
				return false;
			}

			Desc = {};
			Desc.ByteWidth = static_cast<UINT>(Capacity);
			Desc.Usage = D3D11_USAGE_STAGING;
			Desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			if (FAILED(Context->Device->CreateBuffer(&Desc, nullptr, &Readback)))
			{
				View->Release();
				Buffer->Release();
				return false;
			}

			SafeRelease(Context->ReadbackBuffer);
			SafeRelease(Context->OutputView);
			SafeRelease(Context->OutputBuffer);
			Context->OutputBuffer = Buffer;
			Context->OutputView = View;
			Context->ReadbackBuffer = Readback;
			Context->OutputCapacity = Capacity;
			return true;
		}

		bool ValidateBatchedInputs(const float* Matrices, const int* MatrixOffsets,
			const int* VectorOffsets, const int* Dimensions, int IslandCount,
			size_t MatrixElementCount, int VectorElementCount, const float* Bias,
			const float* Lower, const float* Upper, const int* FrictionNormal,
			const float* FrictionCoefficient, const float* Lambda, int MaxIterations,
			float Tolerance, float Relaxation, bool InputsPrevalidated)
		{
			if (IslandCount < 0 || VectorElementCount < 0 || MaxIterations <= 0
				|| !std::isfinite(Tolerance) || Tolerance < 0.0f
				|| !std::isfinite(Relaxation) || Relaxation <= 0.0f || Relaxation >= 2.0f
				|| ((FrictionNormal == nullptr) != (FrictionCoefficient == nullptr)))
			{
				return false;
			}
			if (IslandCount == 0)
			{
				return MatrixElementCount == 0 && VectorElementCount == 0;
			}
			if (!Matrices || !MatrixOffsets || !VectorOffsets || !Dimensions
				|| !Bias || !Lambda)
			{
				return false;
			}

			size_t ExpectedMatrixOffset = 0;
			size_t ExpectedVectorOffset = 0;
			for (int Island = 0; Island < IslandCount; ++Island)
			{
				const int Dimension = Dimensions[Island];
				const int MatrixOffset = MatrixOffsets[Island];
				const int VectorOffset = VectorOffsets[Island];
				if (Dimension <= 0 || MatrixOffset < 0 || VectorOffset < 0
					|| static_cast<size_t>(MatrixOffset) != ExpectedMatrixOffset
					|| static_cast<size_t>(VectorOffset) != ExpectedVectorOffset)
				{
					return false;
				}
				const size_t DimensionSize = static_cast<size_t>(Dimension);
				size_t MatrixSize = 0;
				if (!CheckedMultiply(DimensionSize, DimensionSize, &MatrixSize)
					|| static_cast<size_t>(MatrixOffset) > MatrixElementCount
					|| MatrixSize > MatrixElementCount - static_cast<size_t>(MatrixOffset)
					|| static_cast<size_t>(VectorOffset) > static_cast<size_t>(VectorElementCount)
					|| DimensionSize > static_cast<size_t>(VectorElementCount)
						- static_cast<size_t>(VectorOffset))
				{
					return false;
				}
				if (!CheckedAdd(ExpectedMatrixOffset, MatrixSize, &ExpectedMatrixOffset)
					|| !CheckedAdd(ExpectedVectorOffset, DimensionSize, &ExpectedVectorOffset))
				{
					return false;
				}

				if (InputsPrevalidated)
				{
					continue;
				}
				for (int Row = 0; Row < Dimension; ++Row)
				{
					const size_t GlobalRow = static_cast<size_t>(VectorOffset + Row);
					const float* MatrixRow = Matrices + static_cast<size_t>(MatrixOffset)
						+ static_cast<size_t>(Row) * Dimension;
					if (!std::isfinite(MatrixRow[Row]) || MatrixRow[Row] <= 1.0e-12f
						|| !std::isfinite(Bias[GlobalRow]) || !std::isfinite(Lambda[GlobalRow]))
					{
						return false;
					}
					for (int Column = 0; Column < Dimension; ++Column)
					{
						if (!std::isfinite(MatrixRow[Column]))
						{
							return false;
						}
					}
					const float LowerBound = Lower ? Lower[GlobalRow] : 0.0f;
					const float UpperBound = Upper ? Upper[GlobalRow] : FLT_MAX;
					if (std::isnan(LowerBound) || std::isnan(UpperBound)
						|| LowerBound > UpperBound)
					{
						return false;
					}
					if (FrictionNormal)
					{
						const int NormalRow = FrictionNormal[GlobalRow];
						if (NormalRow < -1 || NormalRow >= Dimension || NormalRow == Row
							|| !std::isfinite(FrictionCoefficient[GlobalRow])
							|| FrictionCoefficient[GlobalRow] < 0.0f)
						{
							return false;
						}
						if (NormalRow >= 0)
						{
							const size_t Normal = static_cast<size_t>(VectorOffset + NormalRow);
							const float NormalLower = Lower ? Lower[Normal] : 0.0f;
							if (FrictionNormal[Normal] >= 0 || NormalLower < 0.0f
								|| LowerBound > 0.0f || UpperBound < 0.0f)
							{
								return false;
							}
						}
					}
				}
			}
			return ExpectedMatrixOffset == MatrixElementCount
				&& ExpectedVectorOffset == static_cast<size_t>(VectorElementCount);
		}

		void UnbindComputeResources(LCPComputeShaderBatchContext* Context)
		{
			ID3D11ShaderResourceView* NullSRV = nullptr;
			ID3D11UnorderedAccessView* NullUAV = nullptr;
			ID3D11Buffer* NullBuffer = nullptr;
			const UINT InitialCount = 0xffffffffu;
			Context->DeviceContext->CSSetShaderResources(0, 1, &NullSRV);
			Context->DeviceContext->CSSetUnorderedAccessViews(0, 1, &NullUAV, &InitialCount);
			Context->DeviceContext->CSSetConstantBuffers(0, 1, &NullBuffer);
			Context->DeviceContext->CSSetShader(nullptr, nullptr, 0);
		}

		bool UpdateConstantsAndDispatch(LCPComputeShaderBatchContext* Context,
			const SolverConstants& Constants, UINT GroupCount)
		{
			D3D11_MAPPED_SUBRESOURCE Mapped = {};
			if (FAILED(Context->DeviceContext->Map(Context->ConstantBuffer, 0,
				D3D11_MAP_WRITE_DISCARD, 0, &Mapped)))
			{
				return false;
			}
			std::memcpy(Mapped.pData, &Constants, sizeof(Constants));
			Context->DeviceContext->Unmap(Context->ConstantBuffer, 0);
			Context->DeviceContext->CSSetConstantBuffers(0, 1, &Context->ConstantBuffer);
			Context->DeviceContext->Dispatch(GroupCount, 1, 1);
			return true;
		}
	}

	LCPComputeShaderBatchContext* CreateLCPComputeShaderBatchContext()
	{
		LCPComputeShaderBatchContext* Context =
			new (std::nothrow) LCPComputeShaderBatchContext();
		if (!Context || !CreateHeadlessDevice(Context)
			|| !CreateShaderAndConstants(Context))
		{
			ReleaseContextResources(Context);
			delete Context;
			return nullptr;
		}
		return Context;
	}

	void DestroyLCPComputeShaderBatchContext(LCPComputeShaderBatchContext* Context)
	{
		ReleaseContextResources(Context);
		delete Context;
	}

	bool IsLCPComputeShaderBackendAvailable()
	{
		static std::once_flag ProbeFlag;
		static bool Available = false;
		std::call_once(ProbeFlag, []()
		{
			LCPComputeShaderBatchContext* Context = CreateLCPComputeShaderBatchContext();
			Available = Context != nullptr;
			DestroyLCPComputeShaderBatchContext(Context);
		});
		return Available;
	}

	bool SolveProjectedGaussSeidelComputeShaderBatched(
		LCPComputeShaderBatchContext* Context, const float* Matrices,
		const int* MatrixOffsets, const int* VectorOffsets, const int* Dimensions,
		int IslandCount, size_t MatrixElementCount, int VectorElementCount,
		const float* Bias, const float* Lower, const float* Upper,
		const int* FrictionNormal, const float* FrictionCoefficient, float* Lambda,
		int MaxIterations, float Tolerance, float Relaxation,
		bool InputsPrevalidated, int* Iterations, float* Residual, bool* Converged)
	{
		for (int Island = 0; Island < std::max(0, IslandCount); ++Island)
		{
			if (Iterations) Iterations[Island] = 0;
			if (Residual) Residual[Island] = 0.0f;
			if (Converged) Converged[Island] = false;
		}
		if (!ValidateBatchedInputs(Matrices, MatrixOffsets, VectorOffsets, Dimensions,
			IslandCount, MatrixElementCount, VectorElementCount, Bias, Lower, Upper,
			FrictionNormal, FrictionCoefficient, Lambda, MaxIterations, Tolerance,
			Relaxation, InputsPrevalidated))
		{
			return false;
		}
		if (IslandCount == 0)
		{
			return true;
		}
		if (!Context || !Context->Device || !Context->DeviceContext
			|| !Context->ComputeShader || !Context->ConstantBuffer)
		{
			return false;
		}

		InputLayout Input = {};
		OutputLayout Output = {};
		if (!BuildLayouts(MatrixElementCount, static_cast<size_t>(VectorElementCount),
			static_cast<size_t>(IslandCount), &Input, &Output)
			|| !EnsureInputCapacity(Context, Input.TotalBytes)
			|| !EnsureOutputCapacity(Context, Output.TotalBytes))
		{
			return false;
		}

		try
		{
			// Some D3D11 drivers may read a complete buffer-sized upload span even
			// when UpdateSubresource is given a byte-range box.
			Context->HostInput.resize(Context->InputCapacity);
			Context->HostOutput.resize(Output.TotalBytes);
		}
		catch (const std::bad_alloc&)
		{
			return false;
		}
		std::fill(Context->HostInput.begin(), Context->HostInput.end(), 0);
		unsigned char* Packed = Context->HostInput.data();
		std::memcpy(Packed + Input.Matrix, Matrices,
			sizeof(float) * MatrixElementCount);
		std::memcpy(Packed + Input.Bias, Bias,
			sizeof(float) * static_cast<size_t>(VectorElementCount));
		std::memcpy(Packed + Input.InitialLambda, Lambda,
			sizeof(float) * static_cast<size_t>(VectorElementCount));
		std::memcpy(Packed + Input.MatrixOffsets, MatrixOffsets,
			sizeof(int) * static_cast<size_t>(IslandCount));
		std::memcpy(Packed + Input.VectorOffsets, VectorOffsets,
			sizeof(int) * static_cast<size_t>(IslandCount));
		std::memcpy(Packed + Input.Dimensions, Dimensions,
			sizeof(int) * static_cast<size_t>(IslandCount));

		float* PackedLower = reinterpret_cast<float*>(Packed + Input.Lower);
		float* PackedUpper = reinterpret_cast<float*>(Packed + Input.Upper);
		float* PackedFrictionCoefficient = reinterpret_cast<float*>(
			Packed + Input.FrictionCoefficient);
		int* PackedFrictionNormal = reinterpret_cast<int*>(Packed + Input.FrictionNormal);
		for (int Row = 0; Row < VectorElementCount; ++Row)
		{
			PackedLower[Row] = Lower ? Lower[Row] : 0.0f;
			PackedUpper[Row] = Upper ? Upper[Row] : FLT_MAX;
			PackedFrictionNormal[Row] = FrictionNormal ? FrictionNormal[Row] : -1;
			PackedFrictionCoefficient[Row] = FrictionCoefficient
				? FrictionCoefficient[Row] : 0.0f;
		}

		D3D11_BOX InputBox = {};
		InputBox.right = static_cast<UINT>(Input.TotalBytes);
		InputBox.bottom = 1;
		InputBox.back = 1;
		Context->DeviceContext->UpdateSubresource(Context->InputBuffer, 0,
			&InputBox, Packed, 0, 0);
		Context->DeviceContext->CSSetShader(Context->ComputeShader, nullptr, 0);
		Context->DeviceContext->CSSetShaderResources(0, 1, &Context->InputView);
		const UINT InitialCount = 0xffffffffu;
		Context->DeviceContext->CSSetUnorderedAccessViews(0, 1,
			&Context->OutputView, &InitialCount);

		SolverConstants Constants = {};
		Constants.MatrixOffsetBytes = static_cast<uint32_t>(Input.Matrix);
		Constants.BiasOffsetBytes = static_cast<uint32_t>(Input.Bias);
		Constants.LowerOffsetBytes = static_cast<uint32_t>(Input.Lower);
		Constants.UpperOffsetBytes = static_cast<uint32_t>(Input.Upper);
		Constants.FrictionCoefficientOffsetBytes = static_cast<uint32_t>(
			Input.FrictionCoefficient);
		Constants.InitialLambdaOffsetBytes = static_cast<uint32_t>(Input.InitialLambda);
		Constants.FrictionNormalOffsetBytes = static_cast<uint32_t>(Input.FrictionNormal);
		Constants.MatrixOffsetsOffsetBytes = static_cast<uint32_t>(Input.MatrixOffsets);
		Constants.VectorOffsetsOffsetBytes = static_cast<uint32_t>(Input.VectorOffsets);
		Constants.DimensionsOffsetBytes = static_cast<uint32_t>(Input.Dimensions);
		Constants.LambdaOutputOffsetBytes = static_cast<uint32_t>(Output.Lambda);
		Constants.ResultsOutputOffsetBytes = static_cast<uint32_t>(Output.Results);
		Constants.MaxIterations = static_cast<uint32_t>(MaxIterations);
		Constants.Tolerance = Tolerance;
		Constants.Relaxation = Relaxation;

		bool Dispatched = true;
		for (int Base = 0; Base < IslandCount; Base += 65535)
		{
			const int Count = std::min(65535, IslandCount - Base);
			Constants.IslandBase = static_cast<uint32_t>(Base);
			Constants.DispatchIslandCount = static_cast<uint32_t>(Count);
			if (!UpdateConstantsAndDispatch(Context, Constants, static_cast<UINT>(Count)))
			{
				Dispatched = false;
				break;
			}
		}
		UnbindComputeResources(Context);
		if (!Dispatched)
		{
			return false;
		}

		D3D11_BOX OutputBox = {};
		OutputBox.right = static_cast<UINT>(Output.TotalBytes);
		OutputBox.bottom = 1;
		OutputBox.back = 1;
		Context->DeviceContext->CopySubresourceRegion(Context->ReadbackBuffer, 0,
			0, 0, 0, Context->OutputBuffer, 0, &OutputBox);
		D3D11_MAPPED_SUBRESOURCE Mapped = {};
		const HRESULT MapResult = Context->DeviceContext->Map(Context->ReadbackBuffer,
			0, D3D11_MAP_READ, 0, &Mapped);
		if (FAILED(MapResult) || !Mapped.pData)
		{
			return false;
		}
		std::memcpy(Context->HostOutput.data(), Mapped.pData, Output.TotalBytes);
		Context->DeviceContext->Unmap(Context->ReadbackBuffer, 0);

		const unsigned char* Readback = Context->HostOutput.data();
		const float* SolvedLambda = reinterpret_cast<const float*>(Readback + Output.Lambda);
		if (!IsFiniteArray(SolvedLambda, static_cast<size_t>(VectorElementCount)))
		{
			return false;
		}
		for (int Island = 0; Island < IslandCount; ++Island)
		{
			const unsigned char* ResultBytes = Readback + Output.Results
				+ static_cast<size_t>(Island) * 16u;
			uint32_t ResultIterations = 0;
			float ResultResidual = 0.0f;
			uint32_t ResultConverged = 0;
			std::memcpy(&ResultIterations, ResultBytes + 0, sizeof(ResultIterations));
			std::memcpy(&ResultResidual, ResultBytes + 4, sizeof(ResultResidual));
			std::memcpy(&ResultConverged, ResultBytes + 8, sizeof(ResultConverged));
			if (ResultIterations != static_cast<uint32_t>(MaxIterations)
				|| !std::isfinite(ResultResidual) || ResultResidual < 0.0f
				|| ResultConverged > 1u)
			{
				return false;
			}
		}

		std::memcpy(Lambda, SolvedLambda,
			sizeof(float) * static_cast<size_t>(VectorElementCount));
		for (int Island = 0; Island < IslandCount; ++Island)
		{
			const unsigned char* ResultBytes = Readback + Output.Results
				+ static_cast<size_t>(Island) * 16u;
			uint32_t ResultIterations = 0;
			float ResultResidual = 0.0f;
			uint32_t ResultConverged = 0;
			std::memcpy(&ResultIterations, ResultBytes + 0, sizeof(ResultIterations));
			std::memcpy(&ResultResidual, ResultBytes + 4, sizeof(ResultResidual));
			std::memcpy(&ResultConverged, ResultBytes + 8, sizeof(ResultConverged));
			if (Iterations) Iterations[Island] = static_cast<int>(ResultIterations);
			if (Residual) Residual[Island] = ResultResidual;
			if (Converged) Converged[Island] = ResultConverged != 0;
		}
		return true;
	}

	bool SolveProjectedGaussSeidelComputeShader(const float* Matrix,
		const float* Bias, const float* Lower, const float* Upper,
		const int* FrictionNormal, const float* FrictionCoefficient, int Dimension,
		float* Lambda, int MaxIterations, float Tolerance, float Relaxation,
		int* Iterations, float* Residual, bool* Converged)
	{
		if (Iterations) *Iterations = 0;
		if (Residual) *Residual = 0.0f;
		if (Converged) *Converged = false;
		if (Dimension < 0 || MaxIterations <= 0
			|| !std::isfinite(Tolerance) || Tolerance < 0.0f
			|| !std::isfinite(Relaxation) || Relaxation <= 0.0f || Relaxation >= 2.0f
			|| ((FrictionNormal == nullptr) != (FrictionCoefficient == nullptr))
			|| (Dimension > 0 && (!Matrix || !Bias || !Lambda))
			|| !IsLCPComputeShaderBackendAvailable())
		{
			return false;
		}
		if (Dimension == 0)
		{
			if (Converged) *Converged = true;
			return true;
		}

		struct ThreadContext
		{
			ThreadContext()
				: Context(CreateLCPComputeShaderBatchContext())
			{
			}
			~ThreadContext()
			{
				DestroyLCPComputeShaderBatchContext(Context);
			}
			LCPComputeShaderBatchContext* Context;
		};
		static thread_local ThreadContext PerThread;
		if (!PerThread.Context)
		{
			return false;
		}

		const int MatrixOffset = 0;
		const int VectorOffset = 0;
		const size_t MatrixElementCount = static_cast<size_t>(Dimension) * Dimension;
		return SolveProjectedGaussSeidelComputeShaderBatched(PerThread.Context,
			Matrix, &MatrixOffset, &VectorOffset, &Dimension, 1, MatrixElementCount,
			Dimension, Bias, Lower, Upper, FrictionNormal, FrictionCoefficient,
			Lambda, MaxIterations, Tolerance, Relaxation, false,
			Iterations, Residual, Converged);
	}

#else
	struct LCPComputeShaderBatchContext
	{
	};

	LCPComputeShaderBatchContext* CreateLCPComputeShaderBatchContext()
	{
		return nullptr;
	}

	void DestroyLCPComputeShaderBatchContext(LCPComputeShaderBatchContext* Context)
	{
		(void)Context;
	}

	bool IsLCPComputeShaderBackendAvailable()
	{
		return false;
	}

	bool SolveProjectedGaussSeidelComputeShaderBatched(
		LCPComputeShaderBatchContext* Context, const float* Matrices,
		const int* MatrixOffsets, const int* VectorOffsets, const int* Dimensions,
		int IslandCount, size_t MatrixElementCount, int VectorElementCount,
		const float* Bias, const float* Lower, const float* Upper,
		const int* FrictionNormal, const float* FrictionCoefficient, float* Lambda,
		int MaxIterations, float Tolerance, float Relaxation,
		bool InputsPrevalidated, int* Iterations, float* Residual, bool* Converged)
	{
		(void)Context; (void)Matrices; (void)MatrixOffsets; (void)VectorOffsets;
		(void)Dimensions; (void)MatrixElementCount; (void)VectorElementCount;
		(void)Bias; (void)Lower; (void)Upper; (void)FrictionNormal;
		(void)FrictionCoefficient; (void)Lambda; (void)MaxIterations;
		(void)Tolerance; (void)Relaxation; (void)InputsPrevalidated;
		for (int Island = 0; Island < std::max(0, IslandCount); ++Island)
		{
			if (Iterations) Iterations[Island] = 0;
			if (Residual) Residual[Island] = 0.0f;
			if (Converged) Converged[Island] = false;
		}
		return false;
	}

	bool SolveProjectedGaussSeidelComputeShader(const float* Matrix,
		const float* Bias, const float* Lower, const float* Upper,
		const int* FrictionNormal, const float* FrictionCoefficient, int Dimension,
		float* Lambda, int MaxIterations, float Tolerance, float Relaxation,
		int* Iterations, float* Residual, bool* Converged)
	{
		(void)Matrix; (void)Bias; (void)Lower; (void)Upper; (void)FrictionNormal;
		(void)FrictionCoefficient; (void)Dimension; (void)Lambda;
		(void)MaxIterations; (void)Tolerance; (void)Relaxation;
		if (Iterations) *Iterations = 0;
		if (Residual) *Residual = 0.0f;
		if (Converged) *Converged = false;
		return false;
	}
#endif
}
