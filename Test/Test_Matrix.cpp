
#include "Test.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <float.h>
#include <vector>

#include "../Src/Maths/Matrix3.h"
#include "../Src/Maths/MatrixMxN.h"
#include "../Src/LinearSystem/DenseMatrix.h"
#include "../Src/LinearSystem/JacobiIteration.h"
#include "../Src/LinearSystem/GaussSeidelIteration.h"
#include "../Src/LinearSystem/ProjectedGaussSeidel.h"
#include "../Src/LinearSystem/LUFactorization.h"
#include "../Src/LinearSystem/SingularValueDecomposition.h"
#include "../Src/LinearSystem/QRDecomposition.h"
#include "../Src/LinearSystem/MoorePenrosePseudoInverse.h"
#include "../Src/LinearSystem/SparseMatrix.h"
#include "../Src/LinearSystem/SparseVector.h"
#include "../Src/LinearSystem/ConjugateGradientSolver.h"
#include "../Src/RigidBodyDynamics/LCPSolver.h"

// using namespace Riemann;

void TestMatrix1()
{
	printf("Running TestMatrix\n");

	Maths::LinearAlgebra::TDenseMatrix<float> M(10);
	for (int i = 0; i < M.GetRows(); ++i)
	for (int j = 0; j < M.GetRows(); ++j)
		M(i, j) = 0.1f * rand() / RAND_MAX;
	for (int i = 0; i < M.GetRows(); ++i)
		M(i, i) += 2.0f;

	Maths::LinearAlgebra::TDenseMatrix<float> invM = (M+M).Inverse();
	Maths::LinearAlgebra::TDenseMatrix<float> Id = (M+M) * invM;
	bool IsId = Id.IsIdentity(1e-5f);
	EXPECT(IsId);

	float det = (2.0f * M).Determinant();
	float detI = invM.Determinant();

	EXPECT(fabsf(det * detI - 0.9999f) < 0.001f);

	Maths::LinearAlgebra::TDenseVector<float>	Vec = M * M.GetCol(0);
	float dp = M.GetRow(0).Dot(M.GetCol(0));
	Maths::LinearAlgebra::TDenseMatrix<float>	Mat = M * M.Transpose().Transpose();

	EXPECT(dp == Vec[0]);
	EXPECT(dp == Mat[0][0]);
	return;
}

void TestMatrix2()
{
	printf("Running TestMatrix2\n");

	Maths::MatrixN<10> M;
	for (int i = 0; i < 10; ++i)
	for (int j = 0; j < 10; ++j)
		M(i, j) = 0.1f * rand() / RAND_MAX;
	for (int i = 0; i < 10; ++i)
		M(i, i) += 2.0f;

	Maths::MatrixN<10> invM = (M + M).Inverse();
	Maths::MatrixN<10> Id = (M + M) * invM;
	bool IsId = Id.IsIdentity(1e-4f);
	EXPECT(IsId);

	float det = (2.0f * M).Determinant();
	float detI = invM.Determinant();

	EXPECT(fabsf(det * detI - 0.9999f) < 0.001f);

	Maths::VectorN<10>	Vec = M * M.GetCol(0);
	float dp = M.GetRow(0).Dot(M.GetCol(0));
	Maths::MatrixN<10>	Mat = M * M.Transpose().Transpose();

	EXPECT(dp == Vec[0]);
	EXPECT(dp == Mat[0][0]);
	return;
}

void TestSparse()
{
	printf("Running TestSparse\n");
	float aaa[] = {0, 1, 0, 1, 0, 0, 0};
	
	Maths::LinearAlgebra::TSparseVector<float> x(aaa, 7);
	
	return;
}

void TestSolve()
{
	printf("Running TestSolve\n");

	Maths::LinearAlgebra::DenseMatrix A(3);
	A[0][0] = 12.0f;
	A[0][1] = 3.0f;
	A[0][2] = 5.0f;
	A(1, 0) = -1.5f;
	A(1, 1) = 6.5f;
	A(1, 2) = 11.5f;
	A[2][0] = 10.0f;
	A[2][1] = -10.0f;
	A[2][2] = 160.1f;

	Maths::LinearAlgebra::DenseVector B(3);
	B[0] = -10.0f;
	B[1] = -7.0f;
	B[2] = 122.0f;

	Maths::LinearAlgebra::DenseVector X(3);

	X.LoadZero();
	Maths::LinearAlgebra::JacobiIteration_CPU<float>::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), 100);
	Maths::LinearAlgebra::DenseVector Y = A * X;
	EXPECT(Y.FuzzyEqual(B, 1e-2f));

	X.LoadZero();
	Maths::LinearAlgebra::GaussSeidelIteration<float>::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), 100);
	Y = A * X;
	EXPECT(Y.FuzzyEqual(B, 1e-2f));

	Maths::LinearAlgebra::DenseMatrix SPD(3);
	SPD[0][0] = 4.0f;
	SPD[0][1] = 1.0f;
	SPD[0][2] = 0.0f;
	SPD[1][0] = 1.0f;
	SPD[1][1] = 3.0f;
	SPD[1][2] = 1.0f;
	SPD[2][0] = 0.0f;
	SPD[2][1] = 1.0f;
	SPD[2][2] = 2.0f;

	Maths::LinearAlgebra::DenseVector BB(3);
	BB[0] = 1.0f;
	BB[1] = 2.0f;
	BB[2] = 3.0f;

	X.LoadZero();
	Maths::LinearAlgebra::ConjugateGradientSolver<float>::Solve(SPD.GetData(), BB.GetData(), X.GetSize(), X.GetData(), 100);
	Y = SPD * X;
	EXPECT(Y.FuzzyEqual(BB, 1e-2f));

	return;
}

void TestDenseMatrixUtilities()
{
	printf("Running TestDenseMatrixUtilities\n");

	Maths::LinearAlgebra::DenseMatrix M(2, 2);
	M[0][0] = -2.0f;
	M[0][1] = 0.0f;
	M[1][0] = 0.0f;
	M[1][1] = 3.0f;
	EXPECT_NEAR(M.L1Norm(), 5.0f, 1e-6f);
	EXPECT_NEAR(M.L2Norm(), sqrtf(13.0f), 1e-6f);
	EXPECT_NEAR(M.LpNorm(2), sqrtf(13.0f), 1e-6f);
	EXPECT_NEAR(M.LpNorm(3), std::pow(35.0f, 1.0f / 3.0f), 1e-6f);

	float upper_data[] = { 1.0f, 2.0f, 0.0f, 3.0f };
	Maths::LinearAlgebra::TDenseMatrix<float> upper(upper_data, 2, 2);
	EXPECT(upper.IsUpperTriangle());
	EXPECT(!upper.IsLowerTriangle());

	float lower_data[] = { 1.0f, 0.0f, 2.0f, 3.0f };
	Maths::LinearAlgebra::TDenseMatrix<float> lower(lower_data, 2, 2);
	EXPECT(lower.IsLowerTriangle());
	EXPECT(!lower.IsUpperTriangle());

	Maths::LinearAlgebra::DenseMatrix zero(2, 3);
	zero.LoadZero();
	EXPECT(zero.IsZero());
}

void TestPGS()
{
	printf("Running TestPGS\n");

	Maths::LinearAlgebra::DenseMatrix A(3);
	A[0][0] = 12.0f;
	A[0][1] = 3.0f;
	A[0][2] = 5.0f;
	A(1, 0) = -1.5f;
	A(1, 1) = 6.5f;
	A(1, 2) = 11.5f;
	A[2][0] = 10.0f;
	A[2][1] = -10.0f;
	A[2][2] = 160.1f;

	Maths::LinearAlgebra::DenseVector B(3);
	B[0] = -10.0f;
	B[1] = -7.0f;
	B[2] = 122.0f;

	Maths::LinearAlgebra::DenseVector X(3);

	X.LoadZero();
	Maths::LinearAlgebra::ProjectedGaussSeidel<float>::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), 50, 1e-10f);
	Maths::LinearAlgebra::DenseVector Y = A * X + B;
	float dp = X.Dot(Y);
	EXPECT(X >= -0.00001f);
	EXPECT(Y >= -0.00001f);
	EXPECT(dp <= 0.00001f);

	Maths::LinearAlgebra::DenseVector x1(3), X2(3);
	x1[0] = 0.0f; x1[1] = -15.0f; x1[2] = 1.0f;
	X2[0] = 1.0f; X2[1] = 10.0f; X2[2] = 3.0f;

	X.LoadZero();
	B[0] = 10.5f;
	B[1] = 17.0f;
	B[2] = 242.0f;
	Maths::LinearAlgebra::ProjectedGaussSeidel<float>::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), x1.GetData(), X2.GetData(), 50);
	Y = A * X + B;
	dp = Y.Dot(X);
	EXPECT(X >= x1);
	EXPECT(X <= X2);
	for (int i = 0; i < X.GetSize(); ++i)
	{
		EXPECT((X[i] == x1[i] && Y[i] > 0) || (X[i] == X2[i] && Y[i] < 0) || (x1[i] < X[i] && X[i] < X2[i] && Y[i] == 0));
	}

	return;
}

void TestBoundedLCP()
{
	printf("Running TestBoundedLCP\n");

	const float matrix[] =
	{
		2.0f, -1.0f, 0.0f,
		-1.0f, 2.0f, 0.0f,
		0.0f, 0.0f, 1.0f,
	};
	const float bias[] = { -1.0f, -1.0f, -4.0f };
	const float lower[] = { 0.0f, 0.0f, -2.0f };
	const float upper[] = { FLT_MAX, FLT_MAX, 2.0f };
	float lambda[] = { 0.0f, 0.0f, 0.0f };

	Riemann::LCPProblem problem;
	problem.Matrix = matrix;
	problem.Bias = bias;
	problem.LowerBounds = lower;
	problem.UpperBounds = upper;
	problem.Dimension = 3;
	Riemann::LCPSolverSettings settings;
	settings.MaxIterations = 64;
	settings.Tolerance = 1.0e-6f;
	Riemann::LCPSolverResult result;
	EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(problem, lambda, settings, &result));
	EXPECT(result.Converged);
	EXPECT(result.Backend == Riemann::SolverBackend::CPU);
	EXPECT_NEAR(lambda[0], 1.0f, 1.0e-4f);
	EXPECT_NEAR(lambda[1], 1.0f, 1.0e-4f);
	EXPECT_NEAR(lambda[2], 2.0f, 1.0e-4f);
	const float cpuLambda[] = { lambda[0], lambda[1], lambda[2] };
	settings.Backend = Riemann::SolverBackend::SIMD;
	lambda[0] = lambda[1] = lambda[2] = 0.0f;
	EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(problem, lambda, settings, &result));
	EXPECT(result.Converged);
	EXPECT(result.Backend == Riemann::SolverBackend::SIMD);
	EXPECT_FALSE(result.UsedFallback);
	for (int Index = 0; Index < 3; ++Index)
	{
		EXPECT_NEAR(lambda[Index], cpuLambda[Index], 1.0e-5f);
	}
	const bool computeAvailable = Riemann::LCPSolver::IsBackendAvailable(
		Riemann::SolverBackend::ComputeShader);
	if (computeAvailable)
	{
		Riemann::LCPSolverSettings computeSettings = settings;
		computeSettings.Backend = Riemann::SolverBackend::ComputeShader;
		computeSettings.AllowBackendFallback = false;
		lambda[0] = lambda[1] = lambda[2] = 0.0f;
		EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(
			problem, lambda, computeSettings, &result));
		EXPECT(result.Converged);
		EXPECT(result.RequestedBackend == Riemann::SolverBackend::ComputeShader);
		EXPECT(result.Backend == Riemann::SolverBackend::ComputeShader);
		EXPECT_FALSE(result.UsedFallback);
		for (int Index = 0; Index < 3; ++Index)
		{
			EXPECT_NEAR(lambda[Index], cpuLambda[Index], 1.0e-4f);
		}
	}
	else
	{
		printf("  ComputeShader bounded LCP skipped: backend unavailable\n");
	}
#if defined(RIEMANN_ENABLE_CUDA)
	settings.Backend = Riemann::SolverBackend::CUDA;
	settings.AllowBackendFallback = true;
	lambda[0] = lambda[1] = lambda[2] = 0.0f;
	EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(problem, lambda, settings, &result));
	EXPECT(result.Converged);
	EXPECT(result.RequestedBackend == Riemann::SolverBackend::CUDA);
	if (result.Backend == Riemann::SolverBackend::CUDA)
	{
		EXPECT_FALSE(result.UsedFallback);
	}
	else
	{
		EXPECT(result.Backend == Riemann::SolverBackend::CPU);
		EXPECT(result.UsedFallback);
	}
	for (int Index = 0; Index < 3; ++Index)
	{
		EXPECT_NEAR(lambda[Index], cpuLambda[Index], 1.0e-4f);
	}
	const bool cudaAvailable = Riemann::LCPSolver::IsBackendAvailable(Riemann::SolverBackend::CUDA);
	settings.AllowBackendFallback = false;
	lambda[0] = lambda[1] = lambda[2] = 0.0f;
	const bool strictCudaSolved = Riemann::LCPSolver::SolveProjectedGaussSeidel(
		problem, lambda, settings, &result);
	if (cudaAvailable)
	{
		EXPECT(strictCudaSolved);
		EXPECT(result.Backend == Riemann::SolverBackend::CUDA);
	}
	else
	{
		EXPECT_FALSE(strictCudaSolved);
	}
#endif
	settings.Backend = Riemann::SolverBackend::SIMD;
	settings.AllowBackendFallback = true;

	const float frictionMatrix[] =
	{
		1.0f, 0.0f,
		0.0f, 1.0f,
	};
	const float frictionBias[] = { -2.0f, -4.0f };
	const float frictionLower[] = { 0.0f, -FLT_MAX };
	const float frictionUpper[] = { FLT_MAX, FLT_MAX };
	const int normalRows[] = { -1, 0 };
	const float friction[] = { 0.0f, 0.5f };
	float frictionLambda[] = { 0.0f, 0.0f };
	problem.Matrix = frictionMatrix;
	problem.Bias = frictionBias;
	problem.LowerBounds = frictionLower;
	problem.UpperBounds = frictionUpper;
	problem.FrictionNormalIndices = normalRows;
	problem.FrictionCoefficients = friction;
	problem.Dimension = 2;
	EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(problem, frictionLambda, settings, &result));
	EXPECT(result.Converged);
	EXPECT_NEAR(frictionLambda[0], 2.0f, 1.0e-5f);
	EXPECT_NEAR(frictionLambda[1], 1.0f, 1.0e-5f);
	if (computeAvailable)
	{
		Riemann::LCPSolverSettings computeSettings = settings;
		computeSettings.Backend = Riemann::SolverBackend::ComputeShader;
		computeSettings.AllowBackendFallback = false;
		frictionLambda[0] = frictionLambda[1] = 0.0f;
		EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(
			problem, frictionLambda, computeSettings, &result));
		EXPECT(result.Converged);
		EXPECT(result.RequestedBackend == Riemann::SolverBackend::ComputeShader);
		EXPECT(result.Backend == Riemann::SolverBackend::ComputeShader);
		EXPECT_FALSE(result.UsedFallback);
		EXPECT_NEAR(frictionLambda[0], 2.0f, 1.0e-4f);
		EXPECT_NEAR(frictionLambda[1], 1.0f, 1.0e-4f);
	}

	const float invalidMatrix[] = { 0.0f };
	const float invalidBias[] = { -1.0f };
	float invalidLambda[] = { 0.0f };
	problem = Riemann::LCPProblem();
	problem.Matrix = invalidMatrix;
	problem.Bias = invalidBias;
	problem.Dimension = 1;
	EXPECT_FALSE(Riemann::LCPSolver::SolveProjectedGaussSeidel(problem, invalidLambda, settings, &result));

	const float invalidFrictionMatrix[] = { 1.0f, 0.0f, 0.0f, 1.0f };
	const float invalidFrictionBias[] = { -1.0f, -1.0f };
	const float invalidFrictionLower[] = { 0.0f, 0.25f };
	const float invalidFrictionUpper[] = { FLT_MAX, FLT_MAX };
	const int invalidFrictionNormals[] = { -1, 0 };
	const float invalidFrictionCoefficients[] = { 0.0f, 0.5f };
	float invalidFrictionLambda[] = { 0.0f, 0.0f };
	problem.Matrix = invalidFrictionMatrix;
	problem.Bias = invalidFrictionBias;
	problem.LowerBounds = invalidFrictionLower;
	problem.UpperBounds = invalidFrictionUpper;
	problem.FrictionNormalIndices = invalidFrictionNormals;
	problem.FrictionCoefficients = invalidFrictionCoefficients;
	problem.Dimension = 2;
	settings.Backend = Riemann::SolverBackend::CPU;
	EXPECT_FALSE(Riemann::LCPSolver::SolveProjectedGaussSeidel(
		problem, invalidFrictionLambda, settings, &result));
}

void TestLCPBackendParity()
{
	printf("Running TestLCPBackendParity\n");

	const int dimension = 8;
	float matrix[dimension * dimension] = {};
	float bias[dimension];
	float cpuLambda[dimension] = {};
	float simdLambda[dimension] = {};
	float computeLambda[dimension] = {};
#if defined(RIEMANN_ENABLE_CUDA)
	float cudaLambda[dimension] = {};
#endif
	for (int row = 0; row < dimension; ++row)
	{
		matrix[row * dimension + row] = 4.0f;
		if (row > 0)
		{
			matrix[row * dimension + row - 1] = -1.0f;
		}
		if (row + 1 < dimension)
		{
			matrix[row * dimension + row + 1] = -1.0f;
		}
		bias[row] = -1.0f - 0.125f * row;
	}

	Riemann::LCPProblem problem;
	problem.Matrix = matrix;
	problem.Bias = bias;
	problem.Dimension = dimension;
	Riemann::LCPSolverSettings settings;
	settings.MaxIterations = 128;
	settings.Tolerance = 1.0e-6f;
	Riemann::LCPSolverResult cpuResult;
	EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(
		problem, cpuLambda, settings, &cpuResult));
	EXPECT(cpuResult.Converged);

	settings.Backend = Riemann::SolverBackend::SIMD;
	Riemann::LCPSolverResult simdResult;
	EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(
		problem, simdLambda, settings, &simdResult));
	EXPECT(simdResult.Converged);
	for (int row = 0; row < dimension; ++row)
	{
		EXPECT_NEAR(simdLambda[row], cpuLambda[row], 2.0e-5f);
		float residual = bias[row];
		for (int column = 0; column < dimension; ++column)
		{
			residual += matrix[row * dimension + column] * simdLambda[column];
		}
		EXPECT(simdLambda[row] >= 0.0f);
		EXPECT(std::fabs(residual) <= 1.0e-4f);
	}

	const bool computeAvailable = Riemann::LCPSolver::IsBackendAvailable(
		Riemann::SolverBackend::ComputeShader);
	if (computeAvailable)
	{
		Riemann::LCPSolverSettings computeSettings = settings;
		computeSettings.Backend = Riemann::SolverBackend::ComputeShader;
		computeSettings.AllowBackendFallback = false;
		Riemann::LCPSolverResult computeResult;
		EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(
			problem, computeLambda, computeSettings, &computeResult));
		EXPECT(computeResult.Converged);
		EXPECT(computeResult.RequestedBackend == Riemann::SolverBackend::ComputeShader);
		EXPECT(computeResult.Backend == Riemann::SolverBackend::ComputeShader);
		EXPECT_FALSE(computeResult.UsedFallback);
		for (int row = 0; row < dimension; ++row)
		{
			EXPECT_NEAR(computeLambda[row], cpuLambda[row], 1.0e-4f);
			float residual = bias[row];
			for (int column = 0; column < dimension; ++column)
			{
				residual += matrix[row * dimension + column] * computeLambda[column];
			}
			EXPECT(computeLambda[row] >= 0.0f);
			EXPECT(std::fabs(residual) <= 1.0e-4f);
		}
	}
	else
	{
		printf("  ComputeShader backend parity skipped: backend unavailable\n");
	}

#if defined(RIEMANN_ENABLE_CUDA)
	settings.Backend = Riemann::SolverBackend::CUDA;
	Riemann::LCPSolverResult cudaResult;
	EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(
		problem, cudaLambda, settings, &cudaResult));
	EXPECT(cudaResult.Converged);
	for (int row = 0; row < dimension; ++row)
	{
		EXPECT_NEAR(cudaLambda[row], cpuLambda[row], 1.0e-4f);
	}
#endif
}

void TestLCPSolverPerformance()
{
	printf("Running TestLCPSolverPerformance\n");
	printf("  Public LCPProblem storage is dense row-major; sparse rows retain explicit zeroes.\n");
	printf("  Times are end-to-end public solver calls, including GPU upload and readback.\n");
	printf("  pattern  size    nnz%%      CPU ms     SIMD ms  Compute ms     CUDA ms   CPU/SIMD  CPU/Compute   CPU/CUDA\n");

	struct BackendTiming
	{
		double MillisecondsPerSolve = 0.0;
		int Iterations = 0;
		bool Available = false;
		std::vector<float> Lambda;
	};

	const int dimensions[] = { 16, 64, 256, 512 };
	const bool sparsePatterns[] = { true, false };
	const Riemann::SolverBackend backends[] = {
		Riemann::SolverBackend::CPU,
		Riemann::SolverBackend::SIMD,
		Riemann::SolverBackend::ComputeShader,
#if defined(RIEMANN_ENABLE_CUDA)
		Riemann::SolverBackend::CUDA,
#endif
	};
	const int backendCount = (int)(sizeof(backends) / sizeof(backends[0]));

	auto PairHash = [](int row, int column) -> uint32_t
	{
		uint32_t value = (uint32_t)(row + 1) * 0x9e3779b9u
			^ (uint32_t)(column + 1) * 0x85ebca6bu;
		value ^= value >> 16;
		value *= 0x7feb352du;
		value ^= value >> 15;
		return value;
	};

	for (bool sparsePattern : sparsePatterns)
	{
		for (int dimension : dimensions)
		{
			std::vector<float> matrix((size_t)dimension * dimension, 0.0f);
			std::vector<float> bias((size_t)dimension, 0.0f);
			std::vector<float> rowAbsoluteSum((size_t)dimension, 0.0f);
			size_t nonZeroCount = 0;
			for (int row = 0; row < dimension; ++row)
			{
				for (int column = row + 1; column < dimension; ++column)
				{
					const uint32_t hash = PairHash(row, column);
					const bool include = !sparsePattern || column == row + 1
						|| hash % 1000u < 40u;
					if (!include)
					{
						continue;
					}
					const float coupling = -0.01f * (float)(1u + hash % 7u);
					matrix[(size_t)row * dimension + column] = coupling;
					matrix[(size_t)column * dimension + row] = coupling;
					rowAbsoluteSum[(size_t)row] += std::fabs(coupling);
					rowAbsoluteSum[(size_t)column] += std::fabs(coupling);
					nonZeroCount += 2;
				}
			}
			for (int row = 0; row < dimension; ++row)
			{
				matrix[(size_t)row * dimension + row] = rowAbsoluteSum[(size_t)row] + 1.0f;
				bias[(size_t)row] = -0.75f - 0.015625f * (float)(row & 15);
				++nonZeroCount;
			}

			Riemann::LCPProblem problem;
			problem.Matrix = matrix.data();
			problem.Bias = bias.data();
			problem.Dimension = dimension;
			Riemann::LCPSolverSettings settings;
			settings.MaxIterations = 128;
			settings.Tolerance = 1.0e-5f;
			settings.AllowBackendFallback = false;

			std::vector<BackendTiming> timings((size_t)backendCount);
			for (int backendIndex = 0; backendIndex < backendCount; ++backendIndex)
			{
				const Riemann::SolverBackend backend = backends[backendIndex];
				BackendTiming& timing = timings[(size_t)backendIndex];
				timing.Available = Riemann::LCPSolver::IsBackendAvailable(backend);
				if (!timing.Available)
				{
					continue;
				}

				settings.Backend = backend;
				timing.Lambda.assign((size_t)dimension, 0.0f);
				Riemann::LCPSolverResult result;
				bool solved = Riemann::LCPSolver::SolveProjectedGaussSeidel(
					problem, timing.Lambda.data(), settings, &result);
				EXPECT(solved);
				EXPECT(result.Converged);
				EXPECT(result.Backend == backend);
				EXPECT_FALSE(result.UsedFallback);

				int repeatCount = 1;
				double elapsedMilliseconds = 0.0;
				do
				{
					const auto begin = std::chrono::steady_clock::now();
					for (int repeat = 0; repeat < repeatCount; ++repeat)
					{
						std::fill(timing.Lambda.begin(), timing.Lambda.end(), 0.0f);
						solved = Riemann::LCPSolver::SolveProjectedGaussSeidel(
							problem, timing.Lambda.data(), settings, &result) && solved;
					}
					elapsedMilliseconds = std::chrono::duration<double, std::milli>(
						std::chrono::steady_clock::now() - begin).count();
					if (elapsedMilliseconds < 8.0 && repeatCount < 256)
					{
						repeatCount *= 2;
					}
					else
					{
						break;
					}
				} while (true);

				EXPECT(solved);
				EXPECT(result.Converged);
				EXPECT(result.RequestedBackend == backend);
				EXPECT(result.Backend == backend);
				EXPECT_FALSE(result.UsedFallback);
				timing.MillisecondsPerSolve = elapsedMilliseconds / repeatCount;
				timing.Iterations = result.Iterations;
			}

			EXPECT(timings[0].Available && timings[1].Available);
			for (int backendIndex = 1; backendIndex < backendCount; ++backendIndex)
			{
				if (!timings[(size_t)backendIndex].Available)
				{
					continue;
				}
				for (int row = 0; row < dimension; ++row)
				{
					EXPECT_NEAR(timings[(size_t)backendIndex].Lambda[(size_t)row],
						timings[0].Lambda[(size_t)row], 2.0e-4f);
				}
			}

			const double nonZeroPercent = 100.0 * (double)nonZeroCount
				/ ((double)dimension * dimension);
			const double cpuMilliseconds = timings[0].MillisecondsPerSolve;
			const double simdMilliseconds = timings[1].MillisecondsPerSolve;
			const bool computeAvailable = timings[2].Available;
			char computeMillisecondsText[32] = "n/a";
			char computeRatioText[32] = "n/a";
			char computeIterationsText[16] = "n/a";
			if (computeAvailable)
			{
				snprintf(computeMillisecondsText, sizeof(computeMillisecondsText), "%.5f",
					timings[2].MillisecondsPerSolve);
				snprintf(computeRatioText, sizeof(computeRatioText), "%.2fx",
					cpuMilliseconds / timings[2].MillisecondsPerSolve);
				snprintf(computeIterationsText, sizeof(computeIterationsText), "%d",
					timings[2].Iterations);
			}
			char cudaMillisecondsText[32] = "n/a";
			char cudaRatioText[32] = "n/a";
			char cudaIterationsText[16] = "n/a";
#if defined(RIEMANN_ENABLE_CUDA)
			const int cudaBackendIndex = 3;
			const bool cudaAvailable = backendCount > cudaBackendIndex
				&& timings[(size_t)cudaBackendIndex].Available;
			if (cudaAvailable)
			{
				snprintf(cudaMillisecondsText, sizeof(cudaMillisecondsText), "%.5f",
					timings[(size_t)cudaBackendIndex].MillisecondsPerSolve);
				snprintf(cudaRatioText, sizeof(cudaRatioText), "%.2fx",
					cpuMilliseconds
					/ timings[(size_t)cudaBackendIndex].MillisecondsPerSolve);
				snprintf(cudaIterationsText, sizeof(cudaIterationsText), "%d",
					timings[(size_t)cudaBackendIndex].Iterations);
			}
#endif
			printf("  %-7s %4d %7.2f %11.5f %11.5f %11s %11s %10.2fx %11s %10s  iter=%d/%d/%s/%s\n",
				sparsePattern ? "sparse" : "dense", dimension, nonZeroPercent,
				cpuMilliseconds, simdMilliseconds, computeMillisecondsText,
				cudaMillisecondsText, cpuMilliseconds / simdMilliseconds,
				computeRatioText, cudaRatioText, timings[0].Iterations,
				timings[1].Iterations, computeIterationsText, cudaIterationsText);
		}
	}
}

void TestSVD()
{
	printf("Running TestSVD\n");
	
	Matrix3 A;
	A[0][0] = 12.0f;
	A[0][1] = 3.0f;
	A[0][2] = 5.0f;
	A(1, 0) = -1.5f;
	A(1, 1) = 6.5f;
	A(1, 2) = 11.5f;
	A[2][0] = 10.0f;
	A[2][1] = -10.0f;
	A[2][2] = 160.1f;
	
	Vector3 s1;
	Matrix3 u1, v1;
	A.SingularValueDecompose(u1, s1, v1);
	Matrix3 AA = u1 * Matrix3(s1) * v1.Transpose();
	
	Vector3 s2;
	Matrix3 u2, v2;
	Maths::LinearAlgebra::SingularValueDecomposition<float>()(A.Data(), 3, 3, u2.Data(), s2.Data(), v2.Data());
	Matrix3 AA2 = u2 * Matrix3(s2) * v2.Transpose();
	
	float me1 = sqrtf((AA - A).L2Norm() / 9);
	float me2 = sqrtf((AA2 - A).L2Norm() / 9);

	EXPECT(Maths::LinearAlgebra::DenseMatrix(u1.Data(), 3, 3).IsOrthogonal(0.2f));	// the algorithm is not good
	EXPECT(Maths::LinearAlgebra::DenseMatrix(v1.Data(), 3, 3).IsOrthogonal(0.2f));	// the algorithm is not good
	EXPECT(Maths::LinearAlgebra::DenseMatrix(u2.Data(), 3, 3).IsOrthogonal());
	EXPECT(Maths::LinearAlgebra::DenseMatrix(v2.Data(), 3, 3).IsOrthogonal());

	EXPECT(me1 < 3.0f);		// the algorithm is not good
	EXPECT(me2 < 0.1f);
	
	return;
}

void TestPseudoInverse()
{
	printf("Running TestPseudoInverse\n");
	
	Maths::LinearAlgebra::TDenseMatrix<float> A(2);
	A[0][0] = 12.0f;
	A[0][1] = 3.0f;
	A[1][0] = 5.0f;
	A[1][1] = 1.0f;
	
	Maths::LinearAlgebra::TDenseMatrix<float> InvA = A.Inverse();
	Maths::LinearAlgebra::TDenseMatrix<float> pInvA = A.PseudoInverse();
	Maths::LinearAlgebra::TDenseMatrix<float> AA = A * pInvA * A;
	
	float *p1 = A.GetData(), *p2 = AA.GetData();
	for (int i = 0; i < 4; ++i)
	{
		EXPECT(fabsf(p1[i] - p2[i]) < 0.01f);
	}
	
	A.LoadIdentity();
	A[1][1] = 0.0f;
	
	pInvA = A.PseudoInverse();
	AA = A * pInvA * A;
	
	p1 = A.GetData();
	p2 = AA.GetData();
	for (int i = 0; i < 4; ++i)
	{
		EXPECT(fabsf(p1[i] - p2[i]) < 0.01f);
	}
	
	return;
}

void TestPolarDecomp()
{
	printf("Running TestPolarDecomp\n");
	
	Matrix3 A;
	A[0][0] = 12.0f;
	A[0][1] = 3.0f;
	A[0][2] = 5.0f;
	A(1, 0) = -1.5f;
	A(1, 1) = 6.5f;
	A(1, 2) = 11.5f;
	A[2][0] = 10.0f;
	A[2][1] = -10.0f;
	A[2][2] = 160.1f;
	
	Matrix3 u1, p1;
	A.PolarDecomposeUP(u1, p1);
	
	Matrix3 AA = u1 * p1;
	float norm = (AA - A).L1Norm() / 9;
	EXPECT(norm < 0.001f);
	
	return;
}

void TestQR()
{
	printf("Running TestQR\n");

	Maths::LinearAlgebra::DenseMatrix A(3, 3);
	A[0][0] = 12.0f;
	A[0][1] = -51.0f;
	A[0][2] = 4.0f;
	A[1][0] = 6.0f;
	A[1][1] = 167.0f;
	A[1][2] = -68.0f;
	A[2][0] = -4.0f;
	A[2][1] = 24.0f;
	A[2][2] = -41.1f;

	Maths::LinearAlgebra::DenseMatrix q, r;
	A.QRDecompose(q, r);
	EXPECT(q.IsOrthogonal(0.00001f));
	EXPECT(r.IsUpperTriangle(0.00001f));

	Maths::LinearAlgebra::DenseMatrix A2 = q * r;
	EXPECT(A.FuzzyEqual(A2, 0.01f));
	return;
}

void TestEigen()
{
	printf("Running TestEigen\n");

	Maths::LinearAlgebra::TDenseMatrix<float> A(4, 4);
	A[0][0] = 54.0f;
	A[0][1] = 30.0f;
	A[0][2] = 49.0f;
	A[0][3] = 28.0f;

	A[1][0] = 30.0f;
	A[1][1] = 50.0f;
	A[1][2] = 8.0f;
	A[1][3] = 44.0f;

	A[2][0] = 49.0f;
	A[2][1] = 8.0f;
	A[2][2] = 46.0f;
	A[2][3] = 16.0f;

	A[3][0] = 28.0f;
	A[3][1] = 44.0f;
	A[3][2] = 16.0f;
	A[3][3] = 12.0f;

	EXPECT(A.IsSymmetric());

	Maths::LinearAlgebra::TDenseVector<float> EigenValues;
	Maths::LinearAlgebra::TDenseMatrix<float> EigenVectors;
	A.EigenDecompose(EigenValues, EigenVectors);

	EXPECT(fabsf(EigenValues[0] - 132.0f) < 1.0f);

	for (int i = 0; i < EigenVectors.GetCols(); ++i)
	{
		Maths::LinearAlgebra::TDenseVector<float> v = EigenVectors.GetCol(i);
		Maths::LinearAlgebra::TDenseVector<float> Av = A * v;
		Maths::LinearAlgebra::TDenseVector<float> Ev = EigenValues[i] * v;
		EXPECT(Av.FuzzyEqual(Ev, 0.001f));
	}
	
	EXPECT((A * EigenVectors).FuzzyEqual(EigenVectors * Maths::LinearAlgebra::TDenseMatrix<float>(EigenValues), 0.001f));

	return;
}

void TestCholesky()
{
	printf("Running TestCholesky\n");
	Maths::LinearAlgebra::TDenseMatrix<float> A(3, 3), L;
	A[0][0] = 1.0f;
	A[0][1] = 0.0f;
	A[0][2] = 1.0f;
	A[1][0] = 0.0f;
	A[1][1] = 2.0f;
	A[1][2] = 0.0f;
	A[2][0] = 1.0f;
	A[2][1] = 0.0f;
	A[2][2] = 3.0f;
	
	EXPECT(A.IsSymmetric());
	EXPECT(A.CholeskyDecompose(L));
	EXPECT(L.IsLowerTriangle());
	EXPECT(A.FuzzyEqual(L * L.Transpose()));
	
	Maths::LinearAlgebra::TDenseVector<float>	B(3), X;
	B[0] = 2.0f;
	B[1] = 2.0f;
	B[2] = 4.0f;
	
	EXPECT(A.SolveCholesky(B, X));
	EXPECT(Maths::FuzzyEqual(X.Dot(X), 3.0f));
	
	return;
}

void TestMatrix()
{
	TestMatrix1();
	TestMatrix2();
	TestDenseMatrixUtilities();
	TestSparse();
	TestSolve();
	TestPGS();
	TestBoundedLCP();
	TestLCPBackendParity();
	TestLCPSolverPerformance();
	TestSVD();
	TestPseudoInverse();
	TestPolarDecomp();
	TestQR();
	TestEigen();
	TestCholesky();
	return;
}
