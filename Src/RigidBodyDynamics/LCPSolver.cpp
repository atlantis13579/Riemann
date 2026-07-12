
#include "LCPSolver.h"

#include "CollidingContact.h"
#include "ConstraintSolver.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../Maths/SIMD.h"

#include <algorithm>
#include <chrono>
#include <stddef.h>
#include <cmath>
#include <cstdio>
#include <float.h>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace Riemann
{
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
	struct LCPComputeShaderBatchContext;
	LCPComputeShaderBatchContext* CreateLCPComputeShaderBatchContext();
	void DestroyLCPComputeShaderBatchContext(LCPComputeShaderBatchContext* Context);
	bool IsLCPComputeShaderBackendAvailable();
	bool SolveProjectedGaussSeidelComputeShader(const float* Matrix, const float* Bias,
		const float* Lower, const float* Upper, const int* FrictionNormal,
		const float* FrictionCoefficient, int Dimension, float* Lambda,
		int MaxIterations, float Tolerance, float Relaxation, int* Iterations,
		float* Residual, bool* Converged);
	bool SolveProjectedGaussSeidelComputeShaderBatched(LCPComputeShaderBatchContext* Context,
		const float* Matrices, const int* MatrixOffsets, const int* VectorOffsets,
		const int* Dimensions, int IslandCount, size_t MatrixElementCount,
		int VectorElementCount, const float* Bias, const float* Lower,
		const float* Upper, const int* FrictionNormal,
		const float* FrictionCoefficient, float* Lambda, int MaxIterations,
		float Tolerance, float Relaxation, bool InputsPrevalidated,
		int* Iterations, float* Residual, bool* Converged);
#endif

#if defined(RIEMANN_ENABLE_CUDA)
	struct LCPCUDABatchContext;
	LCPCUDABatchContext* CreateLCPCUDABatchContext();
	void DestroyLCPCUDABatchContext(LCPCUDABatchContext* Context);
	bool IsLCPCUDABackendAvailable();
	bool SolveProjectedGaussSeidelCUDA(const float* Matrix, const float* Bias,
		const float* Lower, const float* Upper, const int* FrictionNormal,
		const float* FrictionCoefficient, int Dimension, float* Lambda,
		int MaxIterations, float Tolerance, float Relaxation, int* Iterations,
		float* Residual, bool* Converged);
	bool SolveProjectedGaussSeidelCUDABatched(LCPCUDABatchContext* Context,
		const float* Matrices, const int* MatrixOffsets, const int* VectorOffsets,
		const int* Dimensions, int IslandCount, size_t MatrixElementCount,
		int VectorElementCount, const float* Bias, const float* Lower,
		const float* Upper, const int* FrictionNormal,
		const float* FrictionCoefficient, float* Lambda, int MaxIterations,
		float Tolerance, float Relaxation, bool InputsPrevalidated,
		int* Iterations, float* Residual,
		bool* Converged);
#endif

	namespace
	{
		typedef float (*LCPRowDotFunction)(const float* Left, const float* Right, int Count);

		static bool IsDenseGPUBackend(SolverBackend Backend)
		{
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
			if (Backend == SolverBackend::ComputeShader)
			{
				return true;
			}
#endif
#if defined(RIEMANN_ENABLE_CUDA)
			return Backend == SolverBackend::CUDA;
#else
			return false;
#endif
		}

		static float DotRowCPU(const float* Left, const float* Right, int Count)
		{
			float Result = 0.0f;
			for (int Index = 0; Index < Count; ++Index)
			{
				Result += Left[Index] * Right[Index];
			}
			return Result;
		}

		static float DotRowSIMD(const float* Left, const float* Right, int Count)
		{
			F128 Accumulator = F128_Zero();
			int Index = 0;
			for (; Index + 4 <= Count; Index += 4)
			{
				Accumulator = F128_MAdd(F128_LoadU(Left + Index),
					F128_LoadU(Right + Index), Accumulator);
			}

			alignas(16) float Lanes[4];
			F128_StoreA(Accumulator, Lanes);
			float Result = Lanes[0] + Lanes[1] + Lanes[2] + Lanes[3];
			for (; Index < Count; ++Index)
			{
				Result += Left[Index] * Right[Index];
			}
			return Result;
		}

		static float GetLowerBound(const LCPProblem& Problem, const float* Lambda, int Row)
		{
			float Lower = Problem.LowerBounds ? Problem.LowerBounds[Row] : 0.0f;
			if (Problem.FrictionNormalIndices && Problem.FrictionCoefficients)
			{
				const int NormalRow = Problem.FrictionNormalIndices[Row];
				if (NormalRow >= 0)
				{
					const float Limit = Problem.FrictionCoefficients[Row]
						* std::max(0.0f, Lambda[NormalRow]);
					Lower = std::max(Lower, -Limit);
				}
			}
			return Lower;
		}

		static float GetUpperBound(const LCPProblem& Problem, const float* Lambda, int Row)
		{
			float Upper = Problem.UpperBounds ? Problem.UpperBounds[Row] : FLT_MAX;
			if (Problem.FrictionNormalIndices && Problem.FrictionCoefficients)
			{
				const int NormalRow = Problem.FrictionNormalIndices[Row];
				if (NormalRow >= 0)
				{
					const float Limit = Problem.FrictionCoefficients[Row]
						* std::max(0.0f, Lambda[NormalRow]);
					Upper = std::min(Upper, Limit);
				}
			}
			return Upper;
		}

		static float CalculateProjectedResidual(const LCPProblem& Problem, const float* Lambda,
			LCPRowDotFunction RowDot)
		{
			float Residual = 0.0f;
			for (int Row = 0; Row < Problem.Dimension; ++Row)
			{
				const float* MatrixRow = Problem.Matrix + (size_t)Row * Problem.Dimension;
				const float W = Problem.Bias[Row] + RowDot(MatrixRow, Lambda, Problem.Dimension);

				const float Lower = GetLowerBound(Problem, Lambda, Row);
				const float Upper = GetUpperBound(Problem, Lambda, Row);
				const float Projected = std::max(Lower, std::min(Upper,
					Lambda[Row] - W / MatrixRow[Row]));
				Residual = std::max(Residual, std::fabs(Projected - Lambda[Row]));
			}
			return Residual;
		}

		static bool ValidateProblem(const LCPProblem& Problem, const float* Lambda,
			const LCPSolverSettings& Settings)
		{
			if (Problem.Dimension < 0 || Settings.MaxIterations <= 0
				|| !std::isfinite(Settings.Tolerance) || Settings.Tolerance < 0.0f
				|| !std::isfinite(Settings.Relaxation)
				|| Settings.Relaxation <= 0.0f || Settings.Relaxation >= 2.0f)
			{
				return false;
			}
			if (Problem.Dimension == 0)
			{
				return true;
			}
			if (!Problem.Matrix || !Problem.Bias || !Lambda)
			{
				return false;
			}

			const float MinimumDiagonal = 1.0e-12f;
			for (int Row = 0; Row < Problem.Dimension; ++Row)
			{
				const float* MatrixRow = Problem.Matrix + (size_t)Row * Problem.Dimension;
				const float Diagonal = MatrixRow[Row];
				if (!std::isfinite(Diagonal) || Diagonal <= MinimumDiagonal
					|| !std::isfinite(Problem.Bias[Row]) || !std::isfinite(Lambda[Row]))
				{
					return false;
				}
				for (int Column = 0; Column < Problem.Dimension; ++Column)
				{
					if (!std::isfinite(MatrixRow[Column]))
					{
						return false;
					}
				}

				const float Lower = Problem.LowerBounds ? Problem.LowerBounds[Row] : 0.0f;
				const float Upper = Problem.UpperBounds ? Problem.UpperBounds[Row] : FLT_MAX;
				if (std::isnan(Lower) || std::isnan(Upper) || Lower > Upper)
				{
					return false;
				}

				if (Problem.FrictionNormalIndices || Problem.FrictionCoefficients)
				{
					if (!Problem.FrictionNormalIndices || !Problem.FrictionCoefficients)
					{
						return false;
					}
					const int NormalRow = Problem.FrictionNormalIndices[Row];
					if (NormalRow < -1 || NormalRow >= Problem.Dimension || NormalRow == Row
						|| !std::isfinite(Problem.FrictionCoefficients[Row])
						|| Problem.FrictionCoefficients[Row] < 0.0f)
					{
						return false;
					}
					if (NormalRow >= 0)
					{
						const float NormalLower = Problem.LowerBounds
							? Problem.LowerBounds[NormalRow] : 0.0f;
						if (Problem.FrictionNormalIndices[NormalRow] >= 0 || NormalLower < 0.0f
							|| Lower > 0.0f || Upper < 0.0f)
						{
							return false;
						}
					}
				}
			}
			return true;
		}

		static bool SolveProjectedGaussSeidelInternal(const LCPProblem& Problem, float* Lambda,
			const LCPSolverSettings& Settings, LCPRowDotFunction RowDot, LCPSolverResult* Result)
		{
			if (Problem.Dimension == 0)
			{
				Result->Converged = true;
				return true;
			}

			for (int Row = 0; Row < Problem.Dimension; ++Row)
			{
				const float Lower = GetLowerBound(Problem, Lambda, Row);
				const float Upper = GetUpperBound(Problem, Lambda, Row);
				Lambda[Row] = std::max(Lower, std::min(Upper, Lambda[Row]));
			}

			for (int Iteration = 0; Iteration < Settings.MaxIterations; ++Iteration)
			{
				float MaximumDelta = 0.0f;
				for (int Row = 0; Row < Problem.Dimension; ++Row)
				{
					const float* MatrixRow = Problem.Matrix + (size_t)Row * Problem.Dimension;
					const float W = Problem.Bias[Row] + RowDot(MatrixRow, Lambda, Problem.Dimension);
					const float Lower = GetLowerBound(Problem, Lambda, Row);
					const float Upper = GetUpperBound(Problem, Lambda, Row);
					const float OldLambda = Lambda[Row];
					const float Unprojected = OldLambda
						- Settings.Relaxation * W / MatrixRow[Row];
					Lambda[Row] = std::max(Lower, std::min(Upper, Unprojected));
					MaximumDelta = std::max(MaximumDelta, std::fabs(Lambda[Row] - OldLambda));
				}

				Result->Iterations = Iteration + 1;
				if (MaximumDelta <= Settings.Tolerance)
				{
					Result->Residual = CalculateProjectedResidual(Problem, Lambda, RowDot);
					Result->Converged = Result->Residual <= Settings.Tolerance;
					if (Result->Converged)
					{
						return true;
					}
				}
			}

			Result->Residual = CalculateProjectedResidual(Problem, Lambda, RowDot);
			Result->Converged = Result->Residual <= Settings.Tolerance;
			return true;
		}
	}

	bool LCPSolver::SolveProjectedGaussSeidel(const LCPProblem& Problem, float* Lambda,
		const LCPSolverSettings& Settings, LCPSolverResult* Result)
	{
		LCPSolverResult LocalResult;
		LCPSolverResult& Output = Result ? *Result : LocalResult;
		Output = LCPSolverResult();
		Output.RequestedBackend = Settings.Backend;

		SolverBackend Backend = Settings.Backend;
		bool CanDispatch = IsBackendAvailable(Backend);
#if defined(RIEMANN_ENABLE_CUDA)
		CanDispatch = CanDispatch || Backend == SolverBackend::CUDA;
#endif
		if (!CanDispatch)
		{
			if (!Settings.AllowBackendFallback)
			{
				return false;
			}
			Backend = SolverBackend::CPU;
			Output.UsedFallback = true;
		}
		Output.Backend = Backend;

		if (!ValidateProblem(Problem, Lambda, Settings))
		{
			return false;
		}

#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
		if (Backend == SolverBackend::ComputeShader)
		{
			if (SolveProjectedGaussSeidelComputeShader(Problem.Matrix, Problem.Bias,
				Problem.LowerBounds, Problem.UpperBounds, Problem.FrictionNormalIndices,
				Problem.FrictionCoefficients, Problem.Dimension, Lambda,
				Settings.MaxIterations, Settings.Tolerance, Settings.Relaxation,
				&Output.Iterations, &Output.Residual, &Output.Converged))
			{
				return true;
			}
			if (!Settings.AllowBackendFallback)
			{
				return false;
			}
			Backend = SolverBackend::CPU;
			Output.Backend = Backend;
			Output.UsedFallback = true;
			Output.Converged = false;
			Output.Iterations = 0;
			Output.Residual = 0.0f;
		}
#endif

#if defined(RIEMANN_ENABLE_CUDA)
		if (Backend == SolverBackend::CUDA)
		{
			if (SolveProjectedGaussSeidelCUDA(Problem.Matrix, Problem.Bias,
				Problem.LowerBounds, Problem.UpperBounds, Problem.FrictionNormalIndices,
				Problem.FrictionCoefficients, Problem.Dimension, Lambda,
				Settings.MaxIterations, Settings.Tolerance, Settings.Relaxation,
				&Output.Iterations, &Output.Residual, &Output.Converged))
			{
				return true;
			}
			if (!Settings.AllowBackendFallback)
			{
				return false;
			}
			Backend = SolverBackend::CPU;
			Output.Backend = Backend;
			Output.UsedFallback = true;
			Output.Converged = false;
			Output.Iterations = 0;
			Output.Residual = 0.0f;
		}
#endif

		const LCPRowDotFunction RowDot = Backend == SolverBackend::SIMD ? DotRowSIMD : DotRowCPU;
		return SolveProjectedGaussSeidelInternal(Problem, Lambda, Settings, RowDot, &Output);
	}

	bool LCPSolver::IsBackendAvailable(SolverBackend Backend)
	{
		switch (Backend)
		{
		case SolverBackend::CPU:
		case SolverBackend::SIMD:
			return true;
		case SolverBackend::ComputeShader:
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
			return IsLCPComputeShaderBackendAvailable();
#else
			return false;
#endif
#if defined(RIEMANN_ENABLE_CUDA)
		case SolverBackend::CUDA:
			return IsLCPCUDABackendAvailable();
#endif
		default:
			return false;
		}
	}

	namespace
	{
		struct LCPContactRow
		{
			int BodyA;
			int BodyB;
			Vector3 LinearA;
			Vector3 AngularA;
			Vector3 LinearB;
			Vector3 AngularB;
			Vector3 InverseMassLinearA;
			Vector3 InverseMassAngularA;
			Vector3 InverseMassLinearB;
			Vector3 InverseMassAngularB;
			float Bias;
			float LowerBound;
			float UpperBound;
			int FrictionNormalRow;
			float FrictionCoefficient;
		};

		struct LCPContactWriteBack
		{
			Contact* ContactPoint;
			RigidBody* BodyA;
			RigidBody* BodyB;
			int NormalRow;
			int TangentRow;
			int BinormalRow;
			Vector3 InitialImpulse;
		};

		struct PendingDenseContactIsland
		{
			std::vector<LCPContactRow> Rows;
			std::vector<LCPContactWriteBack> WriteBacks;
			std::vector<float> Matrix;
			std::vector<float> Bias;
			std::vector<float> LowerBounds;
			std::vector<float> UpperBounds;
			std::vector<int> FrictionNormalRows;
			std::vector<float> FrictionCoefficients;
			std::vector<float> Lambda;
			float Dt = 0.0f;
		};

		struct ContactPairKey
		{
			RigidBody* Low;
			RigidBody* High;

			bool operator==(const ContactPairKey& Other) const
			{
				return Low == Other.Low && High == Other.High;
			}
		};

		struct ContactPairKeyHash
		{
			size_t operator()(const ContactPairKey& Key) const
			{
				const size_t LowHash = std::hash<RigidBody*>()(Key.Low);
				const size_t HighHash = std::hash<RigidBody*>()(Key.High);
				return LowHash ^ (HighHash + (size_t)0x9e3779b9u
					+ (LowHash << 6) + (LowHash >> 2));
			}
		};

		struct CachedContactImpulse
		{
			Vector3 LocalLow;
			Vector3 LocalHigh;
			Vector3 ImpulseOnHigh;
		};

		struct CachedManifoldImpulses
		{
			std::vector<CachedContactImpulse> Contacts;
			uint64_t LastUsedEpoch = 0;
			uint64_t LastFrameId = 0;
			float LastDt = 0.0f;
		};

		static ContactPairKey MakeContactPairKey(RigidBody* BodyA, RigidBody* BodyB)
		{
			if (std::less<RigidBody*>()(BodyB, BodyA))
			{
				return { BodyB, BodyA };
			}
			return { BodyA, BodyB };
		}

		static void GetCanonicalContactAnchors(const ContactPairKey& Key,
			RigidBody* BodyA, RigidBody* BodyB, const Contact& ContactPoint,
			Vector3* LocalLow, Vector3* LocalHigh)
		{
			const Vector3 LocalA = BodyA->WorldToBodyLocal(ContactPoint.PositionWorldA);
			const Vector3 LocalB = BodyB->WorldToBodyLocal(ContactPoint.PositionWorldB);
			if (BodyA == Key.Low)
			{
				*LocalLow = LocalA;
				*LocalHigh = LocalB;
			}
			else
			{
				*LocalLow = LocalB;
				*LocalHigh = LocalA;
			}
		}

		static float CalculateRowCoupling(const LCPContactRow& Left, const LCPContactRow& Right)
		{
			float Coupling = 0.0f;
			if (Left.BodyA == Right.BodyA)
			{
				Coupling += Left.LinearA.Dot(Right.InverseMassLinearA)
					+ Left.AngularA.Dot(Right.InverseMassAngularA);
			}
			if (Left.BodyA == Right.BodyB)
			{
				Coupling += Left.LinearA.Dot(Right.InverseMassLinearB)
					+ Left.AngularA.Dot(Right.InverseMassAngularB);
			}
			if (Left.BodyB == Right.BodyA)
			{
				Coupling += Left.LinearB.Dot(Right.InverseMassLinearA)
					+ Left.AngularB.Dot(Right.InverseMassAngularA);
			}
			if (Left.BodyB == Right.BodyB)
			{
				Coupling += Left.LinearB.Dot(Right.InverseMassLinearB)
					+ Left.AngularB.Dot(Right.InverseMassAngularB);
			}
			return Coupling;
		}

		static float RegularizeContactDiagonal(float Diagonal)
		{
			const float RelativeSoftness = 1.0e-6f;
			return Diagonal * (1.0f + RelativeSoftness);
		}

		struct SparseLCPMatrix
		{
			std::vector<int> RowOffsets;
			std::vector<int> Columns;
			std::vector<float> Values;
			std::vector<float> Diagonal;
		};

		typedef float (*SparseRowDotFunction)(const SparseLCPMatrix& Matrix,
			const float* Lambda, int Row);

		static float DotSparseRowCPU(const SparseLCPMatrix& Matrix,
			const float* Lambda, int Row)
		{
			float Result = 0.0f;
			for (int Entry = Matrix.RowOffsets[(size_t)Row];
				Entry < Matrix.RowOffsets[(size_t)Row + 1]; ++Entry)
			{
				Result += Matrix.Values[(size_t)Entry]
					* Lambda[(size_t)Matrix.Columns[(size_t)Entry]];
			}
			return Result;
		}

		static float DotSparseRowSIMD(const SparseLCPMatrix& Matrix,
			const float* Lambda, int Row)
		{
			const int End = Matrix.RowOffsets[(size_t)Row + 1];
			int Entry = Matrix.RowOffsets[(size_t)Row];
			F128 Accumulator = F128_Zero();
			for (; Entry + 4 <= End; Entry += 4)
			{
				const int* Columns = Matrix.Columns.data() + Entry;
				const F128 LambdaValues = F128_LoadXYZW(
					Lambda[(size_t)Columns[0]], Lambda[(size_t)Columns[1]],
					Lambda[(size_t)Columns[2]], Lambda[(size_t)Columns[3]]);
				Accumulator = F128_MAdd(F128_LoadU(Matrix.Values.data() + Entry),
					LambdaValues, Accumulator);
			}

			alignas(16) float Lanes[4];
			F128_StoreA(Accumulator, Lanes);
			float Result = Lanes[0] + Lanes[1] + Lanes[2] + Lanes[3];
			for (; Entry < End; ++Entry)
			{
				Result += Matrix.Values[(size_t)Entry]
					* Lambda[(size_t)Matrix.Columns[(size_t)Entry]];
			}
			return Result;
		}

		static bool BuildSparseLCPMatrix(const std::vector<LCPContactRow>& Rows,
			size_t BodyCount, SparseLCPMatrix* Matrix)
		{
			if (Matrix == nullptr)
			{
				return false;
			}

			const int Dimension = (int)Rows.size();
			std::vector<std::vector<int> > RowsByBody(BodyCount);
			for (int RowIndex = 0; RowIndex < Dimension; ++RowIndex)
			{
				const LCPContactRow& Row = Rows[(size_t)RowIndex];
				RowsByBody[(size_t)Row.BodyA].push_back(RowIndex);
				RowsByBody[(size_t)Row.BodyB].push_back(RowIndex);
			}

			Matrix->RowOffsets.assign((size_t)Dimension + 1, 0);
			Matrix->Columns.clear();
			Matrix->Values.clear();
			Matrix->Diagonal.assign((size_t)Dimension, 0.0f);
			std::vector<int> Candidates;
			for (int RowIndex = 0; RowIndex < Dimension; ++RowIndex)
			{
				Matrix->RowOffsets[(size_t)RowIndex] = (int)Matrix->Columns.size();
				const LCPContactRow& Row = Rows[(size_t)RowIndex];
				Candidates = RowsByBody[(size_t)Row.BodyA];
				Candidates.insert(Candidates.end(), RowsByBody[(size_t)Row.BodyB].begin(),
					RowsByBody[(size_t)Row.BodyB].end());
				std::sort(Candidates.begin(), Candidates.end());
				Candidates.erase(std::unique(Candidates.begin(), Candidates.end()), Candidates.end());

				for (int Column : Candidates)
				{
					float Coupling = CalculateRowCoupling(Row, Rows[(size_t)Column]);
					if (!std::isfinite(Coupling))
					{
						return false;
					}
					if (Column == RowIndex)
					{
						Coupling = RegularizeContactDiagonal(Coupling);
						Matrix->Diagonal[(size_t)RowIndex] = Coupling;
					}
					if (Column == RowIndex || Coupling != 0.0f)
					{
						Matrix->Columns.push_back(Column);
						Matrix->Values.push_back(Coupling);
					}
				}

				const float Diagonal = Matrix->Diagonal[(size_t)RowIndex];
				if (!std::isfinite(Diagonal) || Diagonal <= 1.0e-12f)
				{
					return false;
				}
			}
			Matrix->RowOffsets[(size_t)Dimension] = (int)Matrix->Columns.size();
			return true;
		}

		static float CalculateSparseProjectedResidual(const LCPProblem& Problem,
			const SparseLCPMatrix& Matrix, const float* Lambda, SparseRowDotFunction RowDot)
		{
			float Residual = 0.0f;
			for (int Row = 0; Row < Problem.Dimension; ++Row)
			{
				const float W = Problem.Bias[Row] + RowDot(Matrix, Lambda, Row);
				const float Lower = GetLowerBound(Problem, Lambda, Row);
				const float Upper = GetUpperBound(Problem, Lambda, Row);
				const float Projected = std::max(Lower, std::min(Upper,
					Lambda[Row] - W / Matrix.Diagonal[(size_t)Row]));
				Residual = std::max(Residual, std::fabs(Projected - Lambda[Row]));
			}
			return Residual;
		}

		static bool SolveSparseProjectedGaussSeidel(const LCPProblem& Problem,
			const SparseLCPMatrix& Matrix, float* Lambda, const LCPSolverSettings& Settings,
			LCPSolverResult* Result)
		{
			if (Lambda == nullptr || Result == nullptr || Problem.Bias == nullptr
				|| Problem.Dimension <= 0 || Settings.MaxIterations <= 0
				|| !std::isfinite(Settings.Tolerance) || Settings.Tolerance < 0.0f
				|| !std::isfinite(Settings.Relaxation)
				|| Settings.Relaxation <= 0.0f || Settings.Relaxation >= 2.0f
				|| Matrix.Diagonal.size() != (size_t)Problem.Dimension
				|| Matrix.RowOffsets.size() != (size_t)Problem.Dimension + 1
				|| Matrix.Columns.size() != Matrix.Values.size()
				|| ((Problem.FrictionNormalIndices == nullptr)
					!= (Problem.FrictionCoefficients == nullptr)))
			{
				return false;
			}
			for (int Row = 0; Row < Problem.Dimension; ++Row)
			{
				const float Lower = Problem.LowerBounds ? Problem.LowerBounds[Row] : 0.0f;
				const float Upper = Problem.UpperBounds ? Problem.UpperBounds[Row] : FLT_MAX;
				if (!std::isfinite(Matrix.Diagonal[(size_t)Row])
					|| Matrix.Diagonal[(size_t)Row] <= 1.0e-12f
					|| !std::isfinite(Problem.Bias[Row]) || !std::isfinite(Lambda[Row])
					|| std::isnan(Lower) || std::isnan(Upper) || Lower > Upper)
				{
					return false;
				}
				if (Problem.FrictionNormalIndices)
				{
					const int NormalRow = Problem.FrictionNormalIndices[Row];
					if (NormalRow < -1 || NormalRow >= Problem.Dimension || NormalRow == Row
						|| !std::isfinite(Problem.FrictionCoefficients[Row])
						|| Problem.FrictionCoefficients[Row] < 0.0f)
					{
						return false;
					}
					if (NormalRow >= 0)
					{
						const float NormalLower = Problem.LowerBounds
							? Problem.LowerBounds[NormalRow] : 0.0f;
						if (Problem.FrictionNormalIndices[NormalRow] >= 0
							|| NormalLower < 0.0f || Lower > 0.0f || Upper < 0.0f)
						{
							return false;
						}
					}
				}
			}

			*Result = LCPSolverResult();
			Result->RequestedBackend = Settings.Backend;
			Result->Backend = Settings.Backend;
			const SparseRowDotFunction RowDot = Settings.Backend == SolverBackend::SIMD
				? DotSparseRowSIMD : DotSparseRowCPU;
			for (int Row = 0; Row < Problem.Dimension; ++Row)
			{
				const float Lower = GetLowerBound(Problem, Lambda, Row);
				const float Upper = GetUpperBound(Problem, Lambda, Row);
				Lambda[Row] = std::max(Lower, std::min(Upper, Lambda[Row]));
			}

			for (int Iteration = 0; Iteration < Settings.MaxIterations; ++Iteration)
			{
				float MaximumDelta = 0.0f;
				for (int Row = 0; Row < Problem.Dimension; ++Row)
				{
					const float W = Problem.Bias[Row] + RowDot(Matrix, Lambda, Row);
					const float Lower = GetLowerBound(Problem, Lambda, Row);
					const float Upper = GetUpperBound(Problem, Lambda, Row);
					const float OldLambda = Lambda[Row];
					const float Unprojected = OldLambda
						- Settings.Relaxation * W / Matrix.Diagonal[(size_t)Row];
					Lambda[Row] = std::max(Lower, std::min(Upper, Unprojected));
					MaximumDelta = std::max(MaximumDelta,
						std::fabs(Lambda[Row] - OldLambda));
				}

				Result->Iterations = Iteration + 1;
				if (MaximumDelta <= Settings.Tolerance)
				{
					Result->Residual = CalculateSparseProjectedResidual(
						Problem, Matrix, Lambda, RowDot);
					Result->Converged = Result->Residual <= Settings.Tolerance;
					if (Result->Converged)
					{
						return true;
					}
				}
			}

			Result->Residual = CalculateSparseProjectedResidual(Problem, Matrix, Lambda, RowDot);
			Result->Converged = Result->Residual <= Settings.Tolerance;
			return true;
		}

		static float CalculateRowVelocity(const LCPContactRow& Row,
			const std::vector<GeneralizedVelocity>& Velocities)
		{
			const GeneralizedVelocity& VelocityA = Velocities[(size_t)Row.BodyA];
			const GeneralizedVelocity& VelocityB = Velocities[(size_t)Row.BodyB];
			return Row.LinearA.Dot(VelocityA.v) + Row.AngularA.Dot(VelocityA.w)
				+ Row.LinearB.Dot(VelocityB.v) + Row.AngularB.Dot(VelocityB.w);
		}

		static int AppendContactRow(std::vector<LCPContactRow>* Rows, int BodySlotA, int BodySlotB,
			RigidBody* BodyA, RigidBody* BodyB, const Contact& ContactPoint, const Vector3& Direction,
			float Bias, float LowerBound, float UpperBound, int FrictionNormalRow,
			float FrictionCoefficient)
		{
			LCPContactRow Row;
			Row.BodyA = BodySlotA;
			Row.BodyB = BodySlotB;
			Row.LinearA = -Direction;
			Row.AngularA = -ContactPoint.PositionLocalA.Cross(Direction);
			Row.LinearB = Direction;
			Row.AngularB = ContactPoint.PositionLocalB.Cross(Direction);
			Row.InverseMassLinearA = BodyA->GetInverseMass() * Row.LinearA;
			Row.InverseMassAngularA = BodyA->GetInverseInertia_WorldSpace() * Row.AngularA;
			Row.InverseMassLinearB = BodyB->GetInverseMass() * Row.LinearB;
			Row.InverseMassAngularB = BodyB->GetInverseInertia_WorldSpace() * Row.AngularB;
			Row.Bias = Bias;
			Row.LowerBound = LowerBound;
			Row.UpperBound = UpperBound;
			Row.FrictionNormalRow = FrictionNormalRow;
			Row.FrictionCoefficient = FrictionCoefficient;

			const float Diagonal = CalculateRowCoupling(Row, Row);
			if (!std::isfinite(Diagonal) || Diagonal <= 1.0e-12f)
			{
				return -1;
			}
			Rows->push_back(Row);
			return (int)Rows->size() - 1;
		}

		static float CalculateVelocityContactBias(const Contact& ContactPoint, const RigidBody* BodyA,
			const RigidBody* BodyB, const GeneralizedVelocity& VelocityA,
			const GeneralizedVelocity& VelocityB, float Dt)
		{
			const Vector3 RelativeVelocity = VelocityB.v
				+ VelocityB.w.Cross(ContactPoint.PositionLocalB)
				- VelocityA.v - VelocityA.w.Cross(ContactPoint.PositionLocalA);
			float ClosingSpeed = RelativeVelocity.Dot(ContactPoint.Normal);
			const float RestitutionSlop = 1.0f;
			ClosingSpeed = std::min(ClosingSpeed + RestitutionSlop, 0.0f);
			ClosingSpeed = ClosingSpeed < -RestitutionSlop ? ClosingSpeed : 0.0f;

			const float PenetrationSlop = 0.005f;
			const float MaximumDepenetrationVelocity = 0.5f;
			const float Baumgarte = std::max(-MaximumDepenetrationVelocity, -(0.1f / Dt)
				* std::max(ContactPoint.PenetrationDepth - PenetrationSlop, 0.0f));
			const float Restitution = std::max(BodyA->GetRestitution(), BodyB->GetRestitution());
			return Baumgarte + Restitution * ClosingSpeed;
		}

		class GlobalLCPSolver : public ConstraintSolver
		{
		public:
			explicit GlobalLCPSolver(const LCPSolverSettings& Settings)
				: m_Settings(Settings)
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
				, m_ComputeShaderBatchContext(nullptr)
#endif
#if defined(RIEMANN_ENABLE_CUDA)
				, m_CUDABatchContext(nullptr)
#endif
			{
				if (!LCPSolver::IsBackendAvailable(m_Settings.Backend)
					&& m_Settings.AllowBackendFallback)
				{
					m_Settings.Backend = SolverBackend::CPU;
				}
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
				if (m_Settings.Backend == SolverBackend::ComputeShader)
				{
					m_ComputeShaderBatchContext = CreateLCPComputeShaderBatchContext();
					if (!m_ComputeShaderBatchContext && m_Settings.AllowBackendFallback)
					{
						m_Settings.Backend = SolverBackend::CPU;
					}
				}
#endif
#if defined(RIEMANN_ENABLE_CUDA)
				if (m_Settings.Backend == SolverBackend::CUDA)
				{
					m_CUDABatchContext = CreateLCPCUDABatchContext();
					if (!m_CUDABatchContext && m_Settings.AllowBackendFallback)
					{
						m_Settings.Backend = SolverBackend::CPU;
					}
				}
#endif
			}

			~GlobalLCPSolver() override
			{
#if defined(RIEMANN_ENABLE_CUDA)
#if defined(RIEMANN_CUDA_PROFILE)
				if (m_ProfileBatchCalls > 0)
				{
					const double Calls = (double)m_ProfileBatchCalls;
					std::printf("  CUDA host profile calls=%llu islands=%.1f row+matrix=%.3f batchPack=%.3f API=%.3f ms/call\n",
						(unsigned long long)m_ProfileBatchCalls,
						(double)m_ProfileIslandCount / Calls,
						m_ProfileRowBuildMilliseconds / Calls,
						m_ProfileBatchPackMilliseconds / Calls,
						m_ProfileAPIMilliseconds / Calls);
				}
#endif
				DestroyLCPCUDABatchContext(m_CUDABatchContext);
#endif
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
				DestroyLCPComputeShaderBatchContext(m_ComputeShaderBatchContext);
#endif
			}

			SolverBackend GetBackend() const override final
			{
				return m_Settings.Backend;
			}

			void BeginFrame(uint64_t FrameId) override final
			{
				m_FrameId = FrameId;
				++m_CacheEpoch;
			}

			void ResetState() override final
			{
				m_ImpulseCache.clear();
				m_PendingDenseIslands.clear();
			}

			void PreResolve(const std::vector<Geometry*>& Geometries) override final
			{
				m_PendingDenseIslands.clear();
				m_GeometryToBodySlot.assign(Geometries.size(), -1);
				m_Bodies.clear();
				std::unordered_map<RigidBody*, int> BodySlots;
				BodySlots.reserve(Geometries.size());
				for (size_t GeometryIndex = 0; GeometryIndex < Geometries.size(); ++GeometryIndex)
				{
					Geometry* GeometryObject = Geometries[GeometryIndex];
					RigidBody* Body = GeometryObject ? GeometryObject->GetParent<RigidBody>() : nullptr;
					if (!Body)
					{
						continue;
					}

					auto Existing = BodySlots.find(Body);
					if (Existing == BodySlots.end())
					{
						const int BodySlot = (int)m_Bodies.size();
						BodySlots.emplace(Body, BodySlot);
						m_Bodies.push_back(Body);
						m_GeometryToBodySlot[GeometryIndex] = BodySlot;
					}
					else
					{
						m_GeometryToBodySlot[GeometryIndex] = Existing->second;
					}
				}

				m_Velocities.resize(m_Bodies.size());
				for (size_t BodySlot = 0; BodySlot < m_Bodies.size(); ++BodySlot)
				{
					m_Velocities[BodySlot].v = m_Bodies[BodySlot]->GetLinearVelocity();
					m_Velocities[BodySlot].w = m_Bodies[BodySlot]->GetAngularVelocity();
				}

				std::unordered_set<RigidBody*> CurrentBodies(m_Bodies.begin(), m_Bodies.end());
				for (auto It = m_ImpulseCache.begin(); It != m_ImpulseCache.end();)
				{
					const bool BodyRemoved = CurrentBodies.count(It->first.Low) == 0
						|| CurrentBodies.count(It->first.High) == 0;
					const bool CacheExpired = m_CacheEpoch > It->second.LastUsedEpoch + 8;
					if (BodyRemoved || CacheExpired)
					{
						It = m_ImpulseCache.erase(It);
					}
					else
					{
						++It;
					}
				}
			}

			void ResolveContact(const std::vector<Geometry*>& Geometries,
				std::vector<ContactManifold*>& Manifolds, float Dt) override final
			{
				if (Manifolds.empty() || m_Bodies.empty() || Dt <= 0.0f)
				{
					return;
				}
				bool HasActiveBody = false;
				for (ContactManifold* Manifold : Manifolds)
				{
					if (!Manifold || Manifold->indexA < 0 || Manifold->indexB < 0
						|| (size_t)Manifold->indexA >= Geometries.size()
						|| (size_t)Manifold->indexB >= Geometries.size())
					{
						continue;
					}
					RigidBody* BodyA = Geometries[(size_t)Manifold->indexA]
						? Geometries[(size_t)Manifold->indexA]->GetParent<RigidBody>() : nullptr;
					RigidBody* BodyB = Geometries[(size_t)Manifold->indexB]
						? Geometries[(size_t)Manifold->indexB]->GetParent<RigidBody>() : nullptr;
					const RigidBodyDynamic* DynamicA = BodyA ? BodyA->CastDynamic() : nullptr;
					const RigidBodyDynamic* DynamicB = BodyB ? BodyB->CastDynamic() : nullptr;
					HasActiveBody = (BodyA && BodyA->mRigidType != RigidType::Static
						&& (!DynamicA || !DynamicA->Sleeping))
						|| (BodyB && BodyB->mRigidType != RigidType::Static
							&& (!DynamicB || !DynamicB->Sleeping));
					if (HasActiveBody)
					{
						break;
					}
				}
				if (!HasActiveBody)
				{
					return;
				}
#if defined(RIEMANN_ENABLE_CUDA) && defined(RIEMANN_CUDA_PROFILE)
				const auto ProfileRowBuildBegin = std::chrono::steady_clock::now();
#endif

				std::vector<LCPContactRow> Rows;
				std::vector<LCPContactWriteBack> WriteBacks;
				std::unordered_map<ContactPairKey, std::vector<bool>, ContactPairKeyHash>
					UsedCachedContacts;
				Rows.reserve(Manifolds.size() * MAX_CONTACT_POINTS * 3);
				WriteBacks.reserve(Manifolds.size() * MAX_CONTACT_POINTS);
				for (ContactManifold* Manifold : Manifolds)
				{
					if (!Manifold || Manifold->NumContactPointCount <= 0
						|| Manifold->indexA < 0 || Manifold->indexB < 0
						|| (size_t)Manifold->indexA >= Geometries.size()
						|| (size_t)Manifold->indexB >= Geometries.size())
					{
						continue;
					}

					const int BodySlotA = m_GeometryToBodySlot[(size_t)Manifold->indexA];
					const int BodySlotB = m_GeometryToBodySlot[(size_t)Manifold->indexB];
					if (BodySlotA < 0 || BodySlotB < 0 || BodySlotA == BodySlotB
						|| (size_t)BodySlotA >= m_Bodies.size() || (size_t)BodySlotB >= m_Bodies.size())
					{
						continue;
					}

					RigidBody* BodyA = m_Bodies[(size_t)BodySlotA];
					RigidBody* BodyB = m_Bodies[(size_t)BodySlotB];
					if (!BodyA || !BodyB
						|| Geometries[(size_t)Manifold->indexA]->GetParent<RigidBody>() != BodyA
						|| Geometries[(size_t)Manifold->indexB]->GetParent<RigidBody>() != BodyB)
					{
						continue;
					}

					const ContactPairKey PairKey = MakeContactPairKey(BodyA, BodyB);
					auto CachedIt = m_ImpulseCache.find(PairKey);
					std::vector<bool>* UsedContacts = nullptr;
					const bool CanWarmStart = CachedIt != m_ImpulseCache.end()
						&& CachedIt->second.LastFrameId != m_FrameId;
					if (CanWarmStart)
					{
						auto UsedIt = UsedCachedContacts.emplace(PairKey,
							std::vector<bool>(CachedIt->second.Contacts.size(), false)).first;
						UsedContacts = &UsedIt->second;
					}
					const int ContactCount = std::min(Manifold->NumContactPointCount, MAX_CONTACT_POINTS);
					for (int ContactIndex = 0; ContactIndex < ContactCount; ++ContactIndex)
					{
						Contact& ContactPoint = Manifold->ContactPoints[ContactIndex];
						Vector3 InitialImpulse = Vector3::Zero();
						if (CanWarmStart && UsedContacts)
						{
							Vector3 LocalLow;
							Vector3 LocalHigh;
							GetCanonicalContactAnchors(PairKey, BodyA, BodyB, ContactPoint,
								&LocalLow, &LocalHigh);
							const float MaximumAnchorDistanceSquared = 0.02f;
							float BestDistanceSquared = MaximumAnchorDistanceSquared;
							int BestContact = -1;
							for (size_t CachedContactIndex = 0;
								CachedContactIndex < CachedIt->second.Contacts.size(); ++CachedContactIndex)
							{
								if ((*UsedContacts)[CachedContactIndex])
								{
									continue;
								}
								const CachedContactImpulse& Cached =
									CachedIt->second.Contacts[CachedContactIndex];
								const float DistanceSquared = (Cached.LocalLow - LocalLow).SquareLength()
									+ (Cached.LocalHigh - LocalHigh).SquareLength();
								if (DistanceSquared < BestDistanceSquared)
								{
									BestDistanceSquared = DistanceSquared;
									BestContact = (int)CachedContactIndex;
								}
							}
							if (BestContact >= 0)
							{
								(*UsedContacts)[(size_t)BestContact] = true;
								InitialImpulse = CachedIt->second.Contacts[(size_t)BestContact].ImpulseOnHigh;
								if (CachedIt->second.LastDt > 1.0e-8f)
								{
									const float DtScale = std::max(0.25f, std::min(4.0f,
										Dt / CachedIt->second.LastDt));
									InitialImpulse *= DtScale;
								}
								if (BodyB != PairKey.High)
								{
									InitialImpulse = -InitialImpulse;
								}
							}
						}
						const float Bias = CalculateVelocityContactBias(ContactPoint, BodyA, BodyB,
							m_Velocities[(size_t)BodySlotA], m_Velocities[(size_t)BodySlotB], Dt);
						const int NormalRow = AppendContactRow(&Rows, BodySlotA, BodySlotB,
							BodyA, BodyB, ContactPoint, ContactPoint.Normal, Bias,
							0.0f, FLT_MAX, -1, 0.0f);
						if (NormalRow < 0)
						{
							continue;
						}
						// The two independent tangent bounds form an inscribed friction pyramid,
						// so their combined impulse cannot exceed the Coulomb circle radius.
						const float Friction = 0.70710678118f
							* BodyA->GetFrictionDynamic() * BodyB->GetFrictionDynamic();
						const int TangentRow = AppendContactRow(&Rows, BodySlotA, BodySlotB,
							BodyA, BodyB, ContactPoint, ContactPoint.Tangent, 0.0f,
							-FLT_MAX, FLT_MAX, NormalRow, Friction);
						const int BinormalRow = AppendContactRow(&Rows, BodySlotA, BodySlotB,
							BodyA, BodyB, ContactPoint, ContactPoint.Binormal, 0.0f,
							-FLT_MAX, FLT_MAX, NormalRow, Friction);
						WriteBacks.push_back({ &ContactPoint, BodyA, BodyB,
							NormalRow, TangentRow, BinormalRow, InitialImpulse });
					}
				}

				const int Dimension = (int)Rows.size();
				if (Dimension == 0)
				{
					return;
				}
				const bool UseDenseMatrix = IsDenseGPUBackend(m_Settings.Backend);
				std::vector<float> Matrix;
				if (UseDenseMatrix)
				{
					Matrix.assign((size_t)Dimension * Dimension, 0.0f);
				}
				std::vector<float> Bias((size_t)Dimension, 0.0f);
				std::vector<float> LowerBounds((size_t)Dimension, 0.0f);
				std::vector<float> UpperBounds((size_t)Dimension, FLT_MAX);
				std::vector<int> FrictionNormalRows((size_t)Dimension, -1);
				std::vector<float> FrictionCoefficients((size_t)Dimension, 0.0f);
				std::vector<float> Lambda((size_t)Dimension, 0.0f);
				bool DenseInputValid = true;
				const float WarmStartScale = 0.85f;
				for (const LCPContactWriteBack& WriteBack : WriteBacks)
				{
					Lambda[(size_t)WriteBack.NormalRow] = WarmStartScale * std::max(0.0f,
						WriteBack.InitialImpulse.Dot(WriteBack.ContactPoint->Normal));
					if (WriteBack.TangentRow >= 0)
					{
						Lambda[(size_t)WriteBack.TangentRow] = WarmStartScale
							* WriteBack.InitialImpulse.Dot(WriteBack.ContactPoint->Tangent);
					}
					if (WriteBack.BinormalRow >= 0)
					{
						Lambda[(size_t)WriteBack.BinormalRow] = WarmStartScale
							* WriteBack.InitialImpulse.Dot(WriteBack.ContactPoint->Binormal);
					}
				}
				for (int RowIndex = 0; RowIndex < Dimension; ++RowIndex)
				{
					const LCPContactRow& Row = Rows[(size_t)RowIndex];
					Bias[(size_t)RowIndex] = Row.Bias + CalculateRowVelocity(Row, m_Velocities);
					LowerBounds[(size_t)RowIndex] = Row.LowerBound;
					UpperBounds[(size_t)RowIndex] = Row.UpperBound;
					FrictionNormalRows[(size_t)RowIndex] = Row.FrictionNormalRow;
					FrictionCoefficients[(size_t)RowIndex] = Row.FrictionCoefficient;
					DenseInputValid = DenseInputValid
						&& std::isfinite(Bias[(size_t)RowIndex])
						&& std::isfinite(LowerBounds[(size_t)RowIndex])
						&& std::isfinite(UpperBounds[(size_t)RowIndex])
						&& std::isfinite(FrictionCoefficients[(size_t)RowIndex])
						&& std::isfinite(Lambda[(size_t)RowIndex]);
					if (UseDenseMatrix)
					{
						for (int Column = 0; Column <= RowIndex; ++Column)
						{
							float Coupling = CalculateRowCoupling(Row, Rows[(size_t)Column]);
							if (Column == RowIndex)
							{
								Coupling = RegularizeContactDiagonal(Coupling);
							}
							DenseInputValid = DenseInputValid && std::isfinite(Coupling)
								&& (Column != RowIndex || Coupling > 1.0e-12f);
							Matrix[(size_t)RowIndex * Dimension + Column] = Coupling;
							Matrix[(size_t)Column * Dimension + RowIndex] = Coupling;
						}
					}
				}
				if (UseDenseMatrix && !DenseInputValid)
				{
					return;
				}

				if (UseDenseMatrix)
				{
					PendingDenseContactIsland Pending;
					Pending.Rows = std::move(Rows);
					Pending.WriteBacks = std::move(WriteBacks);
					Pending.Matrix = std::move(Matrix);
					Pending.Bias = std::move(Bias);
					Pending.LowerBounds = std::move(LowerBounds);
					Pending.UpperBounds = std::move(UpperBounds);
					Pending.FrictionNormalRows = std::move(FrictionNormalRows);
					Pending.FrictionCoefficients = std::move(FrictionCoefficients);
					Pending.Lambda = std::move(Lambda);
					Pending.Dt = Dt;
					m_PendingDenseIslands.push_back(std::move(Pending));
#if defined(RIEMANN_ENABLE_CUDA) && defined(RIEMANN_CUDA_PROFILE)
					if (m_Settings.Backend == SolverBackend::CUDA)
					{
						m_ProfileRowBuildMilliseconds += std::chrono::duration<double, std::milli>(
							std::chrono::steady_clock::now() - ProfileRowBuildBegin).count();
						++m_ProfileIslandCount;
					}
#endif
					return;
				}

				LCPProblem Problem;
				Problem.Matrix = Matrix.data();
				Problem.Bias = Bias.data();
				Problem.LowerBounds = LowerBounds.data();
				Problem.UpperBounds = UpperBounds.data();
				Problem.FrictionNormalIndices = FrictionNormalRows.data();
				Problem.FrictionCoefficients = FrictionCoefficients.data();
				Problem.Dimension = Dimension;
				LCPSolverResult Result;
				bool Solved = false;
				if (UseDenseMatrix)
				{
					Solved = LCPSolver::SolveProjectedGaussSeidel(
						Problem, Lambda.data(), m_Settings, &Result);
				}
				else
				{
					SparseLCPMatrix SparseMatrix;
					Solved = BuildSparseLCPMatrix(Rows, m_Bodies.size(), &SparseMatrix)
						&& SolveSparseProjectedGaussSeidel(
							Problem, SparseMatrix, Lambda.data(), m_Settings, &Result);
				}
				if (!Solved)
				{
					return;
				}
				if (Result.UsedFallback)
				{
					m_Settings.Backend = Result.Backend;
				}

				ApplySolvedIsland(Rows, WriteBacks, Lambda.data(), Dt);

			}

			void PostResolve(const std::vector<Geometry*>& Geometries) override final
			{
				(void)Geometries;
				SolvePendingDenseIslands();
				for (size_t BodySlot = 0; BodySlot < m_Bodies.size(); ++BodySlot)
				{
					RigidBodyDynamic* DynamicBody = m_Bodies[BodySlot]
						? m_Bodies[BodySlot]->CastDynamic() : nullptr;
					if (!DynamicBody)
					{
						continue;
					}

					DynamicBody->SetLinearVelocity(m_Velocities[BodySlot].v);
					DynamicBody->SetAngularVelocity(m_Velocities[BodySlot].w);
					DynamicBody->SolverV = m_Velocities[BodySlot].v;
					DynamicBody->SolverW = m_Velocities[BodySlot].w;
				}
			}

		private:
			void SolvePendingDenseIslands()
			{
				if (m_PendingDenseIslands.empty())
				{
					return;
				}
#if defined(RIEMANN_ENABLE_CUDA) && defined(RIEMANN_CUDA_PROFILE)
				const auto ProfileBatchPackBegin = std::chrono::steady_clock::now();
#endif

				const int IslandCount = (int)m_PendingDenseIslands.size();
				size_t TotalMatrixElements = 0;
				size_t TotalRows = 0;
				bool CanPack = IslandCount > 0;
				for (const PendingDenseContactIsland& Island : m_PendingDenseIslands)
				{
					CanPack = CanPack && !Island.Rows.empty()
						&& Island.Bias.size() == Island.Rows.size()
						&& Island.LowerBounds.size() == Island.Rows.size()
						&& Island.UpperBounds.size() == Island.Rows.size()
						&& Island.FrictionNormalRows.size() == Island.Rows.size()
						&& Island.FrictionCoefficients.size() == Island.Rows.size()
						&& Island.Lambda.size() == Island.Rows.size();
					if (Island.Matrix.size() > std::numeric_limits<size_t>::max()
						- TotalMatrixElements || Island.Rows.size()
						> std::numeric_limits<size_t>::max() - TotalRows)
					{
						CanPack = false;
						break;
					}
					TotalMatrixElements += Island.Matrix.size();
					TotalRows += Island.Rows.size();
				}
				CanPack = CanPack
					&& TotalMatrixElements <= (size_t)std::numeric_limits<int>::max()
					&& TotalRows <= (size_t)std::numeric_limits<int>::max();

				std::vector<float> Matrices;
				std::vector<float> Bias;
				std::vector<float> LowerBounds;
				std::vector<float> UpperBounds;
				std::vector<int> FrictionNormalRows;
				std::vector<float> FrictionCoefficients;
				std::vector<float> Lambda;
				std::vector<int> MatrixOffsets((size_t)IslandCount, 0);
				std::vector<int> VectorOffsets((size_t)IslandCount, 0);
				std::vector<int> Dimensions((size_t)IslandCount, 0);
				if (CanPack)
				{
					Matrices.resize(TotalMatrixElements);
					Bias.resize(TotalRows);
					LowerBounds.resize(TotalRows);
					UpperBounds.resize(TotalRows);
					FrictionNormalRows.resize(TotalRows);
					FrictionCoefficients.resize(TotalRows);
					Lambda.resize(TotalRows);

					size_t MatrixOffset = 0;
					size_t VectorOffset = 0;
					for (int IslandIndex = 0; IslandIndex < IslandCount; ++IslandIndex)
					{
						const PendingDenseContactIsland& Island =
							m_PendingDenseIslands[(size_t)IslandIndex];
						MatrixOffsets[(size_t)IslandIndex] = (int)MatrixOffset;
						VectorOffsets[(size_t)IslandIndex] = (int)VectorOffset;
						Dimensions[(size_t)IslandIndex] = (int)Island.Rows.size();
						std::copy(Island.Matrix.begin(), Island.Matrix.end(),
							Matrices.begin() + MatrixOffset);
						std::copy(Island.Bias.begin(), Island.Bias.end(),
							Bias.begin() + VectorOffset);
						std::copy(Island.LowerBounds.begin(), Island.LowerBounds.end(),
							LowerBounds.begin() + VectorOffset);
						std::copy(Island.UpperBounds.begin(), Island.UpperBounds.end(),
							UpperBounds.begin() + VectorOffset);
						std::copy(Island.FrictionNormalRows.begin(),
							Island.FrictionNormalRows.end(),
							FrictionNormalRows.begin() + VectorOffset);
						std::copy(Island.FrictionCoefficients.begin(),
							Island.FrictionCoefficients.end(),
							FrictionCoefficients.begin() + VectorOffset);
						std::copy(Island.Lambda.begin(), Island.Lambda.end(),
							Lambda.begin() + VectorOffset);
						MatrixOffset += Island.Matrix.size();
						VectorOffset += Island.Rows.size();
					}
				}

				bool Solved = false;
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
				if (CanPack && m_Settings.Backend == SolverBackend::ComputeShader)
				{
					Solved = SolveProjectedGaussSeidelComputeShaderBatched(
						m_ComputeShaderBatchContext, Matrices.data(), MatrixOffsets.data(),
						VectorOffsets.data(), Dimensions.data(), IslandCount,
						TotalMatrixElements, (int)TotalRows, Bias.data(), LowerBounds.data(),
						UpperBounds.data(), FrictionNormalRows.data(),
						FrictionCoefficients.data(), Lambda.data(), m_Settings.MaxIterations,
						m_Settings.Tolerance, m_Settings.Relaxation, true,
						nullptr, nullptr, nullptr);
				}
#endif
#if defined(RIEMANN_ENABLE_CUDA)
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
				else if (CanPack && m_Settings.Backend == SolverBackend::CUDA)
#else
				if (CanPack && m_Settings.Backend == SolverBackend::CUDA)
#endif
				{
#if defined(RIEMANN_CUDA_PROFILE)
					m_ProfileBatchPackMilliseconds += std::chrono::duration<double, std::milli>(
						std::chrono::steady_clock::now() - ProfileBatchPackBegin).count();
					const auto ProfileAPIBegin = std::chrono::steady_clock::now();
#endif
					Solved = SolveProjectedGaussSeidelCUDABatched(
						m_CUDABatchContext, Matrices.data(), MatrixOffsets.data(),
						VectorOffsets.data(), Dimensions.data(), IslandCount,
						TotalMatrixElements, (int)TotalRows, Bias.data(), LowerBounds.data(),
						UpperBounds.data(), FrictionNormalRows.data(),
						FrictionCoefficients.data(), Lambda.data(), m_Settings.MaxIterations,
						m_Settings.Tolerance, m_Settings.Relaxation, true,
						nullptr, nullptr, nullptr);
#if defined(RIEMANN_CUDA_PROFILE)
					m_ProfileAPIMilliseconds += std::chrono::duration<double, std::milli>(
						std::chrono::steady_clock::now() - ProfileAPIBegin).count();
					++m_ProfileBatchCalls;
#endif
				}
#endif

				bool UsedFallback = false;
				if (!Solved && m_Settings.AllowBackendFallback)
				{
					LCPSolverSettings CPUSettings = m_Settings;
					CPUSettings.Backend = SolverBackend::CPU;
					CPUSettings.AllowBackendFallback = false;
					Solved = true;
					for (PendingDenseContactIsland& Island : m_PendingDenseIslands)
					{
						LCPProblem Problem;
						Problem.Matrix = Island.Matrix.data();
						Problem.Bias = Island.Bias.data();
						Problem.LowerBounds = Island.LowerBounds.data();
						Problem.UpperBounds = Island.UpperBounds.data();
						Problem.FrictionNormalIndices = Island.FrictionNormalRows.data();
						Problem.FrictionCoefficients = Island.FrictionCoefficients.data();
						Problem.Dimension = (int)Island.Rows.size();
						LCPSolverResult Result;
						if (!LCPSolver::SolveProjectedGaussSeidel(
							Problem, Island.Lambda.data(), CPUSettings, &Result))
						{
							Solved = false;
							break;
						}
					}
					UsedFallback = Solved;
					if (UsedFallback)
					{
						m_Settings.Backend = SolverBackend::CPU;
					}
				}

				if (Solved)
				{
					for (int IslandIndex = 0; IslandIndex < IslandCount; ++IslandIndex)
					{
						PendingDenseContactIsland& Island =
							m_PendingDenseIslands[(size_t)IslandIndex];
						const float* IslandLambda = UsedFallback
							? Island.Lambda.data()
							: Lambda.data() + (size_t)VectorOffsets[(size_t)IslandIndex];
						ApplySolvedIsland(Island.Rows, Island.WriteBacks,
							IslandLambda, Island.Dt);
					}
				}
				m_PendingDenseIslands.clear();
			}

			void ApplySolvedIsland(const std::vector<LCPContactRow>& Rows,
				const std::vector<LCPContactWriteBack>& WriteBacks,
				const float* Lambda, float Dt)
			{
				if (!Lambda)
				{
					return;
				}
				for (size_t RowIndex = 0; RowIndex < Rows.size(); ++RowIndex)
				{
					const LCPContactRow& Row = Rows[RowIndex];
					const float Impulse = Lambda[RowIndex];
					m_Velocities[(size_t)Row.BodyA].v += Row.InverseMassLinearA * Impulse;
					m_Velocities[(size_t)Row.BodyA].w += Row.InverseMassAngularA * Impulse;
					m_Velocities[(size_t)Row.BodyB].v += Row.InverseMassLinearB * Impulse;
					m_Velocities[(size_t)Row.BodyB].w += Row.InverseMassAngularB * Impulse;
				}

				for (const LCPContactWriteBack& WriteBack : WriteBacks)
				{
					WriteBack.ContactPoint->totalImpulseNormal = Lambda[(size_t)WriteBack.NormalRow];
					WriteBack.ContactPoint->totalImpulseTangent = WriteBack.TangentRow >= 0
						? Lambda[(size_t)WriteBack.TangentRow] : 0.0f;
					WriteBack.ContactPoint->totalImpulseBinormal = WriteBack.BinormalRow >= 0
						? Lambda[(size_t)WriteBack.BinormalRow] : 0.0f;
				}

				std::unordered_map<ContactPairKey, std::vector<CachedContactImpulse>,
					ContactPairKeyHash> UpdatedImpulses;
				for (const LCPContactWriteBack& WriteBack : WriteBacks)
				{
					const ContactPairKey PairKey = MakeContactPairKey(WriteBack.BodyA, WriteBack.BodyB);
					Vector3 LocalLow;
					Vector3 LocalHigh;
					GetCanonicalContactAnchors(PairKey, WriteBack.BodyA, WriteBack.BodyB,
						*WriteBack.ContactPoint, &LocalLow, &LocalHigh);
					Vector3 Impulse = WriteBack.ContactPoint->Normal
						* WriteBack.ContactPoint->totalImpulseNormal
						+ WriteBack.ContactPoint->Tangent
						* WriteBack.ContactPoint->totalImpulseTangent
						+ WriteBack.ContactPoint->Binormal
						* WriteBack.ContactPoint->totalImpulseBinormal;
					if (WriteBack.BodyB != PairKey.High)
					{
						Impulse = -Impulse;
					}
					UpdatedImpulses[PairKey].push_back({ LocalLow, LocalHigh, Impulse });
				}
				for (auto& Updated : UpdatedImpulses)
				{
					auto CachedIt = m_ImpulseCache.find(Updated.first);
					if (CachedIt != m_ImpulseCache.end()
						&& CachedIt->second.LastFrameId == m_FrameId)
					{
						continue;
					}
					if (CachedIt == m_ImpulseCache.end())
					{
						CachedIt = m_ImpulseCache.emplace(Updated.first,
							CachedManifoldImpulses()).first;
					}
					CachedManifoldImpulses& Cached = CachedIt->second;
					Cached.Contacts.swap(Updated.second);
					Cached.LastUsedEpoch = m_CacheEpoch;
					Cached.LastFrameId = m_FrameId;
					Cached.LastDt = Dt;
				}
			}

			LCPSolverSettings m_Settings;
			std::vector<int> m_GeometryToBodySlot;
			std::vector<RigidBody*> m_Bodies;
			std::vector<GeneralizedVelocity> m_Velocities;
			std::unordered_map<ContactPairKey, CachedManifoldImpulses,
				ContactPairKeyHash> m_ImpulseCache;
			uint64_t m_CacheEpoch = 0;
			uint64_t m_FrameId = 0;
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
			LCPComputeShaderBatchContext* m_ComputeShaderBatchContext;
#endif
			std::vector<PendingDenseContactIsland> m_PendingDenseIslands;
#if defined(RIEMANN_ENABLE_CUDA)
			LCPCUDABatchContext* m_CUDABatchContext;
#if defined(RIEMANN_CUDA_PROFILE)
			uint64_t m_ProfileBatchCalls = 0;
			uint64_t m_ProfileIslandCount = 0;
			double m_ProfileRowBuildMilliseconds = 0.0;
			double m_ProfileBatchPackMilliseconds = 0.0;
			double m_ProfileAPIMilliseconds = 0.0;
#endif
#endif
		};
	}

	ConstraintSolver* CreateGlobalLCPSolver(const LCPSolverSettings& Settings)
	{
		return new GlobalLCPSolver(Settings);
	}
}
