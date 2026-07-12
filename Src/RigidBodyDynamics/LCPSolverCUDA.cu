#if defined(RIEMANN_ENABLE_CUDA)

#include <cuda_runtime.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cfloat>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <new>

namespace Riemann
{
	// The declaration is intentionally opaque to non-CUDA translation units.
	// A world-owned context reuses both host staging and device allocations
	// across frames, so a packed island batch performs no steady-state mallocs.
	struct LCPCUDABatchContext
	{
		LCPCUDABatchContext()
			: DeviceInput(nullptr)
			, DeviceOutput(nullptr)
			, DeviceInputCapacity(0)
			, DeviceOutputCapacity(0)
			, HostInputCapacity(0)
			, HostOutputCapacity(0)
#if defined(RIEMANN_CUDA_PROFILE)
			, ProfileKernelStart(nullptr)
			, ProfileKernelStop(nullptr)
			, ProfileCalls(0)
			, ProfileGrowths(0)
			, ProfileValidationMilliseconds(0.0)
			, ProfilePackMilliseconds(0.0)
			, ProfileH2DMilliseconds(0.0)
			, ProfileKernelMilliseconds(0.0)
			, ProfileKernelWaitMilliseconds(0.0)
			, ProfileD2HMilliseconds(0.0)
			, ProfileUnpackMilliseconds(0.0)
#endif
		{
		}

		unsigned char* DeviceInput;
		unsigned char* DeviceOutput;
		size_t DeviceInputCapacity;
		size_t DeviceOutputCapacity;
		std::unique_ptr<unsigned char[]> HostInput;
		std::unique_ptr<unsigned char[]> HostOutput;
		size_t HostInputCapacity;
		size_t HostOutputCapacity;
#if defined(RIEMANN_CUDA_PROFILE)
		cudaEvent_t ProfileKernelStart;
		cudaEvent_t ProfileKernelStop;
		uint64_t ProfileCalls;
		uint64_t ProfileGrowths;
		double ProfileValidationMilliseconds;
		double ProfilePackMilliseconds;
		double ProfileH2DMilliseconds;
		double ProfileKernelMilliseconds;
		double ProfileKernelWaitMilliseconds;
		double ProfileD2HMilliseconds;
		double ProfileUnpackMilliseconds;
#endif
	};

	bool IsLCPCUDABackendAvailable();

	namespace
	{
		struct CUDASolverResult
		{
			int Iterations;
			float Residual;
			int Converged;
		};

		__device__ __forceinline__ float GetLowerBoundCUDA(const float* Lower,
			const int* FrictionNormal, const float* FrictionCoefficient,
			const float* Lambda, int Row)
		{
			float Bound = Lower ? Lower[Row] : 0.0f;
			if (FrictionNormal && FrictionCoefficient)
			{
				const int NormalRow = FrictionNormal[Row];
				if (NormalRow >= 0)
				{
					const float Limit = FrictionCoefficient[Row]
						* fmaxf(0.0f, Lambda[NormalRow]);
					Bound = fmaxf(Bound, -Limit);
				}
			}
			return Bound;
		}

		__device__ __forceinline__ float GetUpperBoundCUDA(const float* Upper,
			const int* FrictionNormal, const float* FrictionCoefficient,
			const float* Lambda, int Row)
		{
			float Bound = Upper ? Upper[Row] : FLT_MAX;
			if (FrictionNormal && FrictionCoefficient)
			{
				const int NormalRow = FrictionNormal[Row];
				if (NormalRow >= 0)
				{
					const float Limit = FrictionCoefficient[Row]
						* fmaxf(0.0f, Lambda[NormalRow]);
					Bound = fminf(Bound, Limit);
				}
			}
			return Bound;
		}

		__device__ __forceinline__ float WarpReduceSum(float Value)
		{
			const unsigned int Mask = 0xffffffffu;
			for (int Offset = 16; Offset > 0; Offset >>= 1)
			{
				Value += __shfl_down_sync(Mask, Value, Offset);
			}
			return Value;
		}

		__device__ float CalculateBatchedProjectedResidualCUDA(
			const float* Matrix, const float* Bias, const float* Lower,
			const float* Upper, const int* FrictionNormal,
			const float* FrictionCoefficient, int Dimension,
			const float* Lambda, int Lane)
		{
			const unsigned int Mask = 0xffffffffu;
			float Residual = 0.0f;
			for (int Row = 0; Row < Dimension; ++Row)
			{
				const float* MatrixRow = Matrix + static_cast<size_t>(Row) * Dimension;
				float Dot = 0.0f;
				for (int Column = Lane; Column < Dimension; Column += 32)
				{
					Dot += MatrixRow[Column] * Lambda[Column];
				}
				Dot = WarpReduceSum(Dot);
				if (Lane == 0)
				{
					const float W = Bias[Row] + Dot;
					const float LowerBound = GetLowerBoundCUDA(Lower, FrictionNormal,
						FrictionCoefficient, Lambda, Row);
					const float UpperBound = GetUpperBoundCUDA(Upper, FrictionNormal,
						FrictionCoefficient, Lambda, Row);
					const float Projected = fmaxf(LowerBound, fminf(UpperBound,
						Lambda[Row] - W / MatrixRow[Row]));
					Residual = fmaxf(Residual, fabsf(Projected - Lambda[Row]));
				}
				__syncwarp(Mask);
			}
			return __shfl_sync(Mask, Residual, 0);
		}

		// Islands are independent blocks in the global LCP. Each warp owns one
		// island, keeps the PGS row order strictly serial, and only parallelizes
		// the dense dot product inside the current row.
		__global__ void SolveProjectedGaussSeidelBatchedKernel(
			const float* Matrices, const float* Bias, const float* Lower,
			const float* Upper, const float* FrictionCoefficient,
			const float* InitialLambda, const int* FrictionNormal,
			const int* MatrixOffsets, const int* VectorOffsets,
			const int* Dimensions, int IslandCount, float* Lambda,
			int MaxIterations, float Tolerance, float Relaxation,
			CUDASolverResult* Results)
		{
			const int Lane = static_cast<int>(threadIdx.x) & 31;
			const int WarpInBlock = static_cast<int>(threadIdx.x) >> 5;
			const int WarpsPerBlock = static_cast<int>(blockDim.x) >> 5;
			const int Island = static_cast<int>(blockIdx.x) * WarpsPerBlock + WarpInBlock;
			if (Island >= IslandCount)
			{
				return;
			}

			const unsigned int Mask = 0xffffffffu;
			const int Dimension = Dimensions[Island];
			const int MatrixOffset = MatrixOffsets[Island];
			const int VectorOffset = VectorOffsets[Island];
			const float* Matrix = Matrices + MatrixOffset;
			const float* IslandBias = Bias + VectorOffset;
			const float* IslandLower = Lower + VectorOffset;
			const float* IslandUpper = Upper + VectorOffset;
			const float* IslandFrictionCoefficient = FrictionCoefficient + VectorOffset;
			const int* IslandFrictionNormal = FrictionNormal + VectorOffset;
			float* IslandLambda = Lambda + VectorOffset;

			for (int Row = Lane; Row < Dimension; Row += 32)
			{
				IslandLambda[Row] = InitialLambda[VectorOffset + Row];
			}
			__syncwarp(Mask);

			if (Lane == 0)
			{
				Results[Island].Iterations = 0;
				Results[Island].Residual = 0.0f;
				Results[Island].Converged = 0;
			}

			if (Lane == 0)
			{
				for (int Row = 0; Row < Dimension; ++Row)
				{
					const float LowerBound = GetLowerBoundCUDA(IslandLower,
						IslandFrictionNormal, IslandFrictionCoefficient, IslandLambda, Row);
					const float UpperBound = GetUpperBoundCUDA(IslandUpper,
						IslandFrictionNormal, IslandFrictionCoefficient, IslandLambda, Row);
					IslandLambda[Row] = fmaxf(LowerBound,
						fminf(UpperBound, IslandLambda[Row]));
				}
			}
			__syncwarp(Mask);

			for (int Iteration = 0; Iteration < MaxIterations; ++Iteration)
			{
				float MaximumDelta = 0.0f;
				for (int Row = 0; Row < Dimension; ++Row)
				{
					const float* MatrixRow = Matrix + static_cast<size_t>(Row) * Dimension;
					float Dot = 0.0f;
					for (int Column = Lane; Column < Dimension; Column += 32)
					{
						Dot += MatrixRow[Column] * IslandLambda[Column];
					}
					Dot = WarpReduceSum(Dot);
					if (Lane == 0)
					{
						const float W = IslandBias[Row] + Dot;
						const float LowerBound = GetLowerBoundCUDA(IslandLower,
							IslandFrictionNormal, IslandFrictionCoefficient,
							IslandLambda, Row);
						const float UpperBound = GetUpperBoundCUDA(IslandUpper,
							IslandFrictionNormal, IslandFrictionCoefficient,
							IslandLambda, Row);
						const float OldLambda = IslandLambda[Row];
						const float Unprojected = OldLambda
							- Relaxation * W / MatrixRow[Row];
						IslandLambda[Row] = fmaxf(LowerBound,
							fminf(UpperBound, Unprojected));
						MaximumDelta = fmaxf(MaximumDelta,
							fabsf(IslandLambda[Row] - OldLambda));
					}
					__syncwarp(Mask);
				}

				int CheckResidual = 0;
				if (Lane == 0)
				{
					Results[Island].Iterations = Iteration + 1;
					CheckResidual = MaximumDelta <= Tolerance ? 1 : 0;
				}
				CheckResidual = __shfl_sync(Mask, CheckResidual, 0);
				if (CheckResidual)
				{
					const float Residual = CalculateBatchedProjectedResidualCUDA(
						Matrix, IslandBias, IslandLower, IslandUpper,
						IslandFrictionNormal, IslandFrictionCoefficient,
						Dimension, IslandLambda, Lane);
					int Done = 0;
					if (Lane == 0)
					{
						Results[Island].Residual = Residual;
						Results[Island].Converged = Residual <= Tolerance ? 1 : 0;
						Done = Results[Island].Converged;
					}
					Done = __shfl_sync(Mask, Done, 0);
					if (Done)
					{
						return;
					}
				}
			}

			const float Residual = CalculateBatchedProjectedResidualCUDA(
				Matrix, IslandBias, IslandLower, IslandUpper,
				IslandFrictionNormal, IslandFrictionCoefficient,
				Dimension, IslandLambda, Lane);
			if (Lane == 0)
			{
				Results[Island].Residual = Residual;
				Results[Island].Converged = Residual <= Tolerance ? 1 : 0;
			}
		}

		struct CUDABatchInputLayout
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

		struct CUDABatchOutputLayout
		{
			size_t Lambda;
			size_t Results;
			size_t TotalBytes;
		};

		bool AppendArrayBytes(size_t Count, size_t ElementSize, size_t* Cursor,
			size_t* Offset)
		{
			if (!Cursor || !Offset || (Count > 0
				&& ElementSize > std::numeric_limits<size_t>::max() / Count))
			{
				return false;
			}
			const size_t Bytes = Count * ElementSize;
			if (*Cursor > std::numeric_limits<size_t>::max() - Bytes)
			{
				return false;
			}
			*Offset = *Cursor;
			*Cursor += Bytes;
			return true;
		}

		bool BuildCUDABatchLayouts(size_t MatrixElementCount, size_t VectorElementCount,
			size_t IslandCount, CUDABatchInputLayout* Input,
			CUDABatchOutputLayout* Output)
		{
			if (!Input || !Output)
			{
				return false;
			}

			size_t Cursor = 0;
			if (!AppendArrayBytes(MatrixElementCount, sizeof(float), &Cursor, &Input->Matrix)
				|| !AppendArrayBytes(VectorElementCount, sizeof(float), &Cursor, &Input->Bias)
				|| !AppendArrayBytes(VectorElementCount, sizeof(float), &Cursor, &Input->Lower)
				|| !AppendArrayBytes(VectorElementCount, sizeof(float), &Cursor, &Input->Upper)
				|| !AppendArrayBytes(VectorElementCount, sizeof(float), &Cursor,
					&Input->FrictionCoefficient)
				|| !AppendArrayBytes(VectorElementCount, sizeof(float), &Cursor,
					&Input->InitialLambda)
				|| !AppendArrayBytes(VectorElementCount, sizeof(int), &Cursor,
					&Input->FrictionNormal)
				|| !AppendArrayBytes(IslandCount, sizeof(int), &Cursor, &Input->MatrixOffsets)
				|| !AppendArrayBytes(IslandCount, sizeof(int), &Cursor, &Input->VectorOffsets)
				|| !AppendArrayBytes(IslandCount, sizeof(int), &Cursor, &Input->Dimensions))
			{
				return false;
			}
			Input->TotalBytes = Cursor;

			Cursor = 0;
			if (!AppendArrayBytes(VectorElementCount, sizeof(float), &Cursor, &Output->Lambda)
				|| !AppendArrayBytes(IslandCount, sizeof(CUDASolverResult), &Cursor,
					&Output->Results))
			{
				return false;
			}
			Output->TotalBytes = Cursor;
			return true;
		}

		size_t CalculateGrowthCapacity(size_t Required)
		{
			size_t Capacity = 256;
			while (Capacity < Required)
			{
				if (Capacity > std::numeric_limits<size_t>::max() / 2)
				{
					return Required;
				}
				Capacity *= 2;
			}
			return Capacity;
		}

		bool GrowHostBuffer(std::unique_ptr<unsigned char[]>* Buffer,
			size_t* Capacity, size_t Required)
		{
			if (!Buffer || !Capacity)
			{
				return false;
			}
			if (Required <= *Capacity)
			{
				return true;
			}
			const size_t NewCapacity = CalculateGrowthCapacity(Required);
			std::unique_ptr<unsigned char[]> NewBuffer(
				new (std::nothrow) unsigned char[NewCapacity]);
			if (!NewBuffer)
			{
				return false;
			}
			Buffer->swap(NewBuffer);
			*Capacity = NewCapacity;
			return true;
		}

		bool GrowDeviceBuffer(unsigned char** Buffer, size_t* Capacity, size_t Required)
		{
			if (!Buffer || !Capacity)
			{
				return false;
			}
			if (Required <= *Capacity)
			{
				return true;
			}

			const size_t NewCapacity = CalculateGrowthCapacity(Required);
			void* RawPointer = nullptr;
			if (cudaMalloc(&RawPointer, NewCapacity) != cudaSuccess)
			{
				return false;
			}
			unsigned char* NewBuffer = static_cast<unsigned char*>(RawPointer);
			if (*Buffer && cudaFree(*Buffer) != cudaSuccess)
			{
				(void)cudaFree(NewBuffer);
				return false;
			}
			*Buffer = NewBuffer;
			*Capacity = NewCapacity;
			return true;
		}

		bool EnsureCUDABatchCapacity(LCPCUDABatchContext* Context,
			size_t InputBytes, size_t OutputBytes)
		{
#if defined(RIEMANN_CUDA_PROFILE)
			if (Context && (InputBytes > Context->DeviceInputCapacity
				|| OutputBytes > Context->DeviceOutputCapacity))
			{
				++Context->ProfileGrowths;
			}
#endif
			return Context
				&& GrowHostBuffer(&Context->HostInput, &Context->HostInputCapacity, InputBytes)
				&& GrowHostBuffer(&Context->HostOutput, &Context->HostOutputCapacity, OutputBytes)
				&& GrowDeviceBuffer(&Context->DeviceInput,
					&Context->DeviceInputCapacity, InputBytes)
				&& GrowDeviceBuffer(&Context->DeviceOutput,
					&Context->DeviceOutputCapacity, OutputBytes);
		}

		bool ValidateBatchedCUDAInputs(const float* Matrices,
			const int* MatrixOffsets, const int* VectorOffsets, const int* Dimensions,
			int IslandCount, size_t MatrixElementCount, int VectorElementCount,
			const float* Bias, const float* Lower, const float* Upper,
			const int* FrictionNormal, const float* FrictionCoefficient,
			const float* Lambda, int MaxIterations, float Tolerance, float Relaxation,
			bool InputsPrevalidated)
		{
			if (IslandCount < 0 || VectorElementCount < 0
				|| MatrixElementCount > static_cast<size_t>(std::numeric_limits<int>::max())
				|| MaxIterations <= 0
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
			int ExpectedVectorOffset = 0;
			const float MinimumDiagonal = 1.0e-12f;
			for (int Island = 0; Island < IslandCount; ++Island)
			{
				const int Dimension = Dimensions[Island];
				if (Dimension <= 0 || MatrixOffsets[Island] < 0
					|| static_cast<size_t>(MatrixOffsets[Island]) != ExpectedMatrixOffset
					|| VectorOffsets[Island] != ExpectedVectorOffset
					|| Dimension > std::numeric_limits<int>::max() - ExpectedVectorOffset)
				{
					return false;
				}
				const size_t Size = static_cast<size_t>(Dimension);
				if (ExpectedMatrixOffset > MatrixElementCount
					|| Size > std::numeric_limits<size_t>::max() / Size
					|| Size * Size > MatrixElementCount - ExpectedMatrixOffset)
				{
					return false;
				}

				if (!InputsPrevalidated)
				{
					const float* Matrix = Matrices + ExpectedMatrixOffset;
					for (int LocalRow = 0; LocalRow < Dimension; ++LocalRow)
					{
						const int Row = ExpectedVectorOffset + LocalRow;
						const float* MatrixRow = Matrix
							+ static_cast<size_t>(LocalRow) * Dimension;
						if (!std::isfinite(MatrixRow[LocalRow])
							|| MatrixRow[LocalRow] <= MinimumDiagonal
							|| !std::isfinite(Bias[Row]) || !std::isfinite(Lambda[Row]))
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

						const float LowerBound = Lower ? Lower[Row] : 0.0f;
						const float UpperBound = Upper ? Upper[Row] : FLT_MAX;
						if (std::isnan(LowerBound) || std::isnan(UpperBound)
							|| LowerBound > UpperBound)
						{
							return false;
						}
						if (FrictionNormal)
						{
							const int NormalRow = FrictionNormal[Row];
							if (NormalRow < -1 || NormalRow >= Dimension
								|| NormalRow == LocalRow
								|| !std::isfinite(FrictionCoefficient[Row])
								|| FrictionCoefficient[Row] < 0.0f)
							{
								return false;
							}
							if (NormalRow >= 0)
							{
								const int NormalIndex = ExpectedVectorOffset + NormalRow;
								const float NormalLower = Lower ? Lower[NormalIndex] : 0.0f;
								if (FrictionNormal[NormalIndex] >= 0 || NormalLower < 0.0f
									|| LowerBound > 0.0f || UpperBound < 0.0f)
								{
									return false;
								}
							}
						}
					}
				}

				ExpectedMatrixOffset += Size * Size;
				ExpectedVectorOffset += Dimension;
			}
			return ExpectedMatrixOffset == MatrixElementCount
				&& ExpectedVectorOffset == VectorElementCount;
		}

		bool ValidateCUDAInputs(const float* Matrix, const float* Bias,
			const float* Lower, const float* Upper, const int* FrictionNormal,
			const float* FrictionCoefficient, int Dimension, const float* Lambda,
			int MaxIterations, float Tolerance, float Relaxation)
		{
			if (Dimension < 0 || MaxIterations <= 0 || !std::isfinite(Tolerance)
				|| Tolerance < 0.0f || !std::isfinite(Relaxation)
				|| Relaxation <= 0.0f || Relaxation >= 2.0f)
			{
				return false;
			}
			if (Dimension == 0)
			{
				return true;
			}
			if (!Matrix || !Bias || !Lambda
				|| ((FrictionNormal == nullptr) != (FrictionCoefficient == nullptr)))
			{
				return false;
			}

			const size_t Size = static_cast<size_t>(Dimension);
			if (Size > std::numeric_limits<size_t>::max() / Size
				|| Size * Size > std::numeric_limits<size_t>::max() / sizeof(float))
			{
				return false;
			}

			const float MinimumDiagonal = 1.0e-12f;
			for (int Row = 0; Row < Dimension; ++Row)
			{
				const float* MatrixRow = Matrix + static_cast<size_t>(Row) * Dimension;
				if (!std::isfinite(MatrixRow[Row]) || MatrixRow[Row] <= MinimumDiagonal
					|| !std::isfinite(Bias[Row]) || !std::isfinite(Lambda[Row]))
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

				const float LowerBound = Lower ? Lower[Row] : 0.0f;
				const float UpperBound = Upper ? Upper[Row] : FLT_MAX;
				if (std::isnan(LowerBound) || std::isnan(UpperBound)
					|| LowerBound > UpperBound)
				{
					return false;
				}

				if (FrictionNormal)
				{
					const int NormalRow = FrictionNormal[Row];
					if (NormalRow < -1 || NormalRow >= Dimension || NormalRow == Row
						|| !std::isfinite(FrictionCoefficient[Row])
						|| FrictionCoefficient[Row] < 0.0f)
					{
						return false;
					}
					if (NormalRow >= 0)
					{
						const float NormalLower = Lower ? Lower[NormalRow] : 0.0f;
						if (FrictionNormal[NormalRow] >= 0 || NormalLower < 0.0f
							|| LowerBound > 0.0f || UpperBound < 0.0f)
						{
							return false;
						}
					}
				}
			}
			return true;
		}
	}

	LCPCUDABatchContext* CreateLCPCUDABatchContext()
	{
		LCPCUDABatchContext* Context = new (std::nothrow) LCPCUDABatchContext();
#if defined(RIEMANN_CUDA_PROFILE)
		if (Context && (cudaEventCreate(&Context->ProfileKernelStart) != cudaSuccess
			|| cudaEventCreate(&Context->ProfileKernelStop) != cudaSuccess))
		{
			if (Context->ProfileKernelStart)
			{
				(void)cudaEventDestroy(Context->ProfileKernelStart);
			}
			if (Context->ProfileKernelStop)
			{
				(void)cudaEventDestroy(Context->ProfileKernelStop);
			}
			delete Context;
			return nullptr;
		}
#endif
		return Context;
	}

	void DestroyLCPCUDABatchContext(LCPCUDABatchContext* Context)
	{
		if (!Context)
		{
			return;
		}
#if defined(RIEMANN_CUDA_PROFILE)
		if (Context->ProfileCalls > 0)
		{
			const double Calls = (double)Context->ProfileCalls;
			std::printf("  CUDA API profile calls=%llu growths=%llu validation=%.3f pack=%.3f H2D=%.3f kernel=%.3f wait=%.3f D2H=%.3f unpack=%.3f ms/call\n",
				(unsigned long long)Context->ProfileCalls,
				(unsigned long long)Context->ProfileGrowths,
				Context->ProfileValidationMilliseconds / Calls,
				Context->ProfilePackMilliseconds / Calls,
				Context->ProfileH2DMilliseconds / Calls,
				Context->ProfileKernelMilliseconds / Calls,
				Context->ProfileKernelWaitMilliseconds / Calls,
				Context->ProfileD2HMilliseconds / Calls,
				Context->ProfileUnpackMilliseconds / Calls);
		}
		if (Context->ProfileKernelStart)
		{
			(void)cudaEventDestroy(Context->ProfileKernelStart);
		}
		if (Context->ProfileKernelStop)
		{
			(void)cudaEventDestroy(Context->ProfileKernelStop);
		}
#endif
		if (Context->DeviceOutput)
		{
			(void)cudaFree(Context->DeviceOutput);
		}
		if (Context->DeviceInput)
		{
			(void)cudaFree(Context->DeviceInput);
		}
		delete Context;
	}

	bool SolveProjectedGaussSeidelCUDABatched(LCPCUDABatchContext* Context,
		const float* Matrices, const int* MatrixOffsets, const int* VectorOffsets,
		const int* Dimensions, int IslandCount, size_t MatrixElementCount,
		int VectorElementCount, const float* Bias, const float* Lower,
		const float* Upper, const int* FrictionNormal,
		const float* FrictionCoefficient, float* Lambda, int MaxIterations,
		float Tolerance, float Relaxation, bool InputsPrevalidated,
		int* Iterations, float* Residual,
		bool* Converged)
	{
		if (IslandCount > 0)
		{
			for (int Island = 0; Island < IslandCount; ++Island)
			{
				if (Iterations)
				{
					Iterations[Island] = 0;
				}
				if (Residual)
				{
					Residual[Island] = 0.0f;
				}
				if (Converged)
				{
					Converged[Island] = false;
				}
			}
		}

#if defined(RIEMANN_CUDA_PROFILE)
		const auto ProfileValidationBegin = std::chrono::steady_clock::now();
#endif
		if (!ValidateBatchedCUDAInputs(Matrices, MatrixOffsets, VectorOffsets,
			Dimensions, IslandCount, MatrixElementCount, VectorElementCount,
			Bias, Lower, Upper, FrictionNormal, FrictionCoefficient, Lambda,
			MaxIterations, Tolerance, Relaxation, InputsPrevalidated))
		{
			return false;
		}
		if (IslandCount == 0)
		{
			return true;
		}
		if (!Context || !IsLCPCUDABackendAvailable())
		{
			return false;
		}
#if defined(RIEMANN_CUDA_PROFILE)
		Context->ProfileValidationMilliseconds += std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - ProfileValidationBegin).count();
		const auto ProfilePackBegin = std::chrono::steady_clock::now();
#endif

		CUDABatchInputLayout InputLayout = {};
		CUDABatchOutputLayout OutputLayout = {};
		if (!BuildCUDABatchLayouts(MatrixElementCount,
			static_cast<size_t>(VectorElementCount), static_cast<size_t>(IslandCount),
			&InputLayout, &OutputLayout)
			|| !EnsureCUDABatchCapacity(Context,
				InputLayout.TotalBytes, OutputLayout.TotalBytes))
		{
			return false;
		}

		unsigned char* HostInput = Context->HostInput.get();
		std::memcpy(HostInput + InputLayout.Matrix, Matrices,
			sizeof(float) * MatrixElementCount);
		std::memcpy(HostInput + InputLayout.Bias, Bias,
			sizeof(float) * static_cast<size_t>(VectorElementCount));
		std::memcpy(HostInput + InputLayout.InitialLambda, Lambda,
			sizeof(float) * static_cast<size_t>(VectorElementCount));
		std::memcpy(HostInput + InputLayout.MatrixOffsets, MatrixOffsets,
			sizeof(int) * static_cast<size_t>(IslandCount));
		std::memcpy(HostInput + InputLayout.VectorOffsets, VectorOffsets,
			sizeof(int) * static_cast<size_t>(IslandCount));
		std::memcpy(HostInput + InputLayout.Dimensions, Dimensions,
			sizeof(int) * static_cast<size_t>(IslandCount));

		float* HostLower = reinterpret_cast<float*>(HostInput + InputLayout.Lower);
		float* HostUpper = reinterpret_cast<float*>(HostInput + InputLayout.Upper);
		float* HostFrictionCoefficient = reinterpret_cast<float*>(
			HostInput + InputLayout.FrictionCoefficient);
		int* HostFrictionNormal = reinterpret_cast<int*>(
			HostInput + InputLayout.FrictionNormal);
		for (int Row = 0; Row < VectorElementCount; ++Row)
		{
			HostLower[Row] = Lower ? Lower[Row] : 0.0f;
			HostUpper[Row] = Upper ? Upper[Row] : FLT_MAX;
			HostFrictionNormal[Row] = FrictionNormal ? FrictionNormal[Row] : -1;
			HostFrictionCoefficient[Row] = FrictionCoefficient
				? FrictionCoefficient[Row] : 0.0f;
		}

#if defined(RIEMANN_CUDA_PROFILE)
		Context->ProfilePackMilliseconds += std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - ProfilePackBegin).count();
		const auto ProfileH2DBegin = std::chrono::steady_clock::now();
#endif
		if (cudaMemcpy(Context->DeviceInput, HostInput, InputLayout.TotalBytes,
			cudaMemcpyHostToDevice) != cudaSuccess)
		{
			return false;
		}
#if defined(RIEMANN_CUDA_PROFILE)
		Context->ProfileH2DMilliseconds += std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - ProfileH2DBegin).count();
#endif

		const unsigned char* DeviceInput = Context->DeviceInput;
		const float* DeviceMatrices = reinterpret_cast<const float*>(
			DeviceInput + InputLayout.Matrix);
		const float* DeviceBias = reinterpret_cast<const float*>(
			DeviceInput + InputLayout.Bias);
		const float* DeviceLower = reinterpret_cast<const float*>(
			DeviceInput + InputLayout.Lower);
		const float* DeviceUpper = reinterpret_cast<const float*>(
			DeviceInput + InputLayout.Upper);
		const float* DeviceFrictionCoefficient = reinterpret_cast<const float*>(
			DeviceInput + InputLayout.FrictionCoefficient);
		const float* DeviceInitialLambda = reinterpret_cast<const float*>(
			DeviceInput + InputLayout.InitialLambda);
		const int* DeviceFrictionNormal = reinterpret_cast<const int*>(
			DeviceInput + InputLayout.FrictionNormal);
		const int* DeviceMatrixOffsets = reinterpret_cast<const int*>(
			DeviceInput + InputLayout.MatrixOffsets);
		const int* DeviceVectorOffsets = reinterpret_cast<const int*>(
			DeviceInput + InputLayout.VectorOffsets);
		const int* DeviceDimensions = reinterpret_cast<const int*>(
			DeviceInput + InputLayout.Dimensions);
		float* DeviceLambda = reinterpret_cast<float*>(
			Context->DeviceOutput + OutputLayout.Lambda);
		CUDASolverResult* DeviceResults = reinterpret_cast<CUDASolverResult*>(
			Context->DeviceOutput + OutputLayout.Results);

		const int ThreadsPerBlock = 128;
		const int WarpsPerBlock = ThreadsPerBlock / 32;
		const int BlockCount = (IslandCount + WarpsPerBlock - 1) / WarpsPerBlock;
#if defined(RIEMANN_CUDA_PROFILE)
		(void)cudaEventRecord(Context->ProfileKernelStart);
#endif
		SolveProjectedGaussSeidelBatchedKernel<<<BlockCount, ThreadsPerBlock>>>(
			DeviceMatrices, DeviceBias, DeviceLower, DeviceUpper,
			DeviceFrictionCoefficient, DeviceInitialLambda, DeviceFrictionNormal,
			DeviceMatrixOffsets, DeviceVectorOffsets, DeviceDimensions, IslandCount,
			DeviceLambda, MaxIterations, Tolerance, Relaxation, DeviceResults);
		if (cudaGetLastError() != cudaSuccess)
		{
			return false;
		}
#if defined(RIEMANN_CUDA_PROFILE)
		(void)cudaEventRecord(Context->ProfileKernelStop);
		const auto ProfileKernelWaitBegin = std::chrono::steady_clock::now();
		if (cudaEventSynchronize(Context->ProfileKernelStop) != cudaSuccess)
		{
			return false;
		}
		Context->ProfileKernelWaitMilliseconds += std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - ProfileKernelWaitBegin).count();
		float ProfileKernelMilliseconds = 0.0f;
		if (cudaEventElapsedTime(&ProfileKernelMilliseconds,
			Context->ProfileKernelStart, Context->ProfileKernelStop) != cudaSuccess)
		{
			return false;
		}
		Context->ProfileKernelMilliseconds += ProfileKernelMilliseconds;
		const auto ProfileD2HBegin = std::chrono::steady_clock::now();
#endif
		if (cudaMemcpy(Context->HostOutput.get(), Context->DeviceOutput,
			OutputLayout.TotalBytes, cudaMemcpyDeviceToHost) != cudaSuccess)
		{
			return false;
		}
#if defined(RIEMANN_CUDA_PROFILE)
		Context->ProfileD2HMilliseconds += std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - ProfileD2HBegin).count();
		const auto ProfileUnpackBegin = std::chrono::steady_clock::now();
#endif

		const unsigned char* HostOutput = Context->HostOutput.get();
		std::memcpy(Lambda, HostOutput + OutputLayout.Lambda,
			sizeof(float) * static_cast<size_t>(VectorElementCount));
		const CUDASolverResult* HostResults = reinterpret_cast<const CUDASolverResult*>(
			HostOutput + OutputLayout.Results);
		for (int Island = 0; Island < IslandCount; ++Island)
		{
			if (Iterations)
			{
				Iterations[Island] = HostResults[Island].Iterations;
			}
			if (Residual)
			{
				Residual[Island] = HostResults[Island].Residual;
			}
			if (Converged)
			{
				Converged[Island] = HostResults[Island].Converged != 0;
			}
		}
#if defined(RIEMANN_CUDA_PROFILE)
		Context->ProfileUnpackMilliseconds += std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - ProfileUnpackBegin).count();
		++Context->ProfileCalls;
#endif
		return true;
	}

	bool IsLCPCUDABackendAvailable()
	{
		int DeviceCount = 0;
		const cudaError_t Status = cudaGetDeviceCount(&DeviceCount);
		if (Status != cudaSuccess)
		{
			// Clear the per-thread runtime error so probing does not contaminate
			// a later CUDA operation if a device or driver becomes available.
			(void)cudaGetLastError();
			return false;
		}
		return DeviceCount > 0;
	}

	bool SolveProjectedGaussSeidelCUDA(const float* Matrix, const float* Bias,
		const float* Lower, const float* Upper, const int* FrictionNormal,
		const float* FrictionCoefficient, int Dimension, float* Lambda,
		int MaxIterations, float Tolerance, float Relaxation, int* Iterations,
		float* Residual, bool* Converged)
	{
		if (Iterations)
		{
			*Iterations = 0;
		}
		if (Residual)
		{
			*Residual = 0.0f;
		}
		if (Converged)
		{
			*Converged = false;
		}

		if (!ValidateCUDAInputs(Matrix, Bias, Lower, Upper, FrictionNormal,
			FrictionCoefficient, Dimension, Lambda, MaxIterations, Tolerance, Relaxation)
			|| !IsLCPCUDABackendAvailable())
		{
			return false;
		}
		if (Dimension == 0)
		{
			if (Converged)
			{
				*Converged = true;
			}
			return true;
		}

		struct ThreadBatchContext
		{
			ThreadBatchContext()
				: Context(CreateLCPCUDABatchContext())
			{
			}

			~ThreadBatchContext()
			{
				DestroyLCPCUDABatchContext(Context);
			}

			LCPCUDABatchContext* Context;
		};
		static thread_local ThreadBatchContext ThreadContext;
		if (!ThreadContext.Context)
		{
			return false;
		}

		const int MatrixOffset = 0;
		const int VectorOffset = 0;
		const size_t MatrixElementCount = (size_t)Dimension * Dimension;
		return SolveProjectedGaussSeidelCUDABatched(ThreadContext.Context,
			Matrix, &MatrixOffset, &VectorOffset, &Dimension, 1,
			MatrixElementCount, Dimension, Bias, Lower, Upper, FrictionNormal,
			FrictionCoefficient, Lambda, MaxIterations, Tolerance, Relaxation,
			true, Iterations, Residual, Converged);
	}
}

#endif
