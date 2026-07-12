#include "Test.h"

#if defined(RIEMANN_ENABLE_CUDA)

#include "RigidBodyDynamics/LCPSolver.h"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <vector>

namespace
{
	void TestCUDABoundedLCP()
	{
		printf("Running TestCUDABoundedLCP\n");

		EXPECT(Riemann::LCPSolver::IsBackendAvailable(Riemann::SolverBackend::CUDA));

		const int dimension = 12;
		std::vector<float> matrix(dimension * dimension, 0.0f);
		std::vector<float> bias(dimension, 0.0f);
		std::vector<float> lower(dimension, 0.0f);
		std::vector<float> upper(dimension, FLT_MAX);
		std::vector<int> frictionNormal(dimension, -1);
		std::vector<float> frictionCoefficient(dimension, 0.0f);
		std::vector<float> cpuLambda(dimension, 0.0f);
		std::vector<float> cudaLambda(dimension, 0.0f);

		for (int row = 0; row < dimension; ++row)
		{
			matrix[row * dimension + row] = 4.0f;
			if (row > 0)
			{
				matrix[row * dimension + row - 1] = -0.75f;
			}
			if (row + 1 < dimension)
			{
				matrix[row * dimension + row + 1] = -0.75f;
			}
			bias[row] = -1.0f - 0.0625f * row;
		}

		// Exercise the dynamic friction bounds in the CUDA kernel as well as
		// ordinary unilateral rows. The final row is clamped by row zero.
		lower[dimension - 1] = -FLT_MAX;
		frictionNormal[dimension - 1] = 0;
		frictionCoefficient[dimension - 1] = 0.5f;

		Riemann::LCPProblem problem;
		problem.Matrix = matrix.data();
		problem.Bias = bias.data();
		problem.LowerBounds = lower.data();
		problem.UpperBounds = upper.data();
		problem.FrictionNormalIndices = frictionNormal.data();
		problem.FrictionCoefficients = frictionCoefficient.data();
		problem.Dimension = dimension;

		Riemann::LCPSolverSettings settings;
		settings.MaxIterations = 256;
		settings.Tolerance = 1.0e-6f;
		settings.Backend = Riemann::SolverBackend::CPU;
		Riemann::LCPSolverResult cpuResult;
		EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(
			problem, cpuLambda.data(), settings, &cpuResult));
		EXPECT(cpuResult.Converged);

		settings.Backend = Riemann::SolverBackend::CUDA;
		settings.AllowBackendFallback = false;
		Riemann::LCPSolverResult cudaResult;
		EXPECT(Riemann::LCPSolver::SolveProjectedGaussSeidel(
			problem, cudaLambda.data(), settings, &cudaResult));
		EXPECT(cudaResult.Converged);
		EXPECT(cudaResult.RequestedBackend == Riemann::SolverBackend::CUDA);
		EXPECT(cudaResult.Backend == Riemann::SolverBackend::CUDA);
		EXPECT_FALSE(cudaResult.UsedFallback);

		for (int row = 0; row < dimension; ++row)
		{
			EXPECT_NEAR(cudaLambda[row], cpuLambda[row], 1.0e-4f);
			EXPECT(std::isfinite(cudaLambda[row]));
		}

		const float frictionLimit = 0.5f * cudaLambda[0];
		EXPECT(cudaLambda[dimension - 1] >= -frictionLimit - 1.0e-5f);
		EXPECT(cudaLambda[dimension - 1] <= frictionLimit + 1.0e-5f);
	}
}

void TestCUDA()
{
	TestCUDABoundedLCP();
}

#endif
