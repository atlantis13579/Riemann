#pragma once

#include "SolverBackend.h"
#include <vector>

namespace Riemann
{
	class ConstraintSolver;
	// A mixed LCP extends the non-negative LCP with per-row lower and upper
	// bounds. Friction rows may additionally reference a normal row so their
	// bounds become [-coefficient * normalLambda, coefficient * normalLambda].
	struct LCPProblem
	{
		LCPProblem()
			: Matrix(nullptr)
			, Bias(nullptr)
			, LowerBounds(nullptr)
			, UpperBounds(nullptr)
			, FrictionNormalIndices(nullptr)
			, FrictionCoefficients(nullptr)
			, Dimension(0)
		{
		}

		const float*	Matrix;
		const float*	Bias;
		const float*	LowerBounds;
		const float*	UpperBounds;
		const int*	FrictionNormalIndices;
		const float*	FrictionCoefficients;
		int			Dimension;
	};

	struct LCPSolverSettings
	{
		LCPSolverSettings()
			: MaxIterations(32)
			, Tolerance(1.0e-5f)
			, Relaxation(1.0f)
			, Backend(SolverBackend::CPU)
			, AllowBackendFallback(true)
		{
		}

		int		MaxIterations;
		float	Tolerance;
		float	Relaxation;
		SolverBackend Backend;
		bool	AllowBackendFallback;
	};

	struct LCPSolverResult
	{
		LCPSolverResult()
			: Converged(false)
			, Iterations(0)
			, Residual(0.0f)
			, RequestedBackend(SolverBackend::CPU)
			, Backend(SolverBackend::CPU)
			, UsedFallback(false)
		{
		}

		bool	Converged;
		int		Iterations;
		float	Residual;
		SolverBackend RequestedBackend;
		SolverBackend Backend;
		bool	UsedFallback;
	};

	struct LCPSparseMatrix
	{
		// Compressed sparse rows; Diagonal stores each row's positive pivot.
		std::vector<int> RowOffsets;
		std::vector<int> Columns;
		std::vector<float> Values;
		std::vector<float> Diagonal;
	};

	class LCPSolver
	{
	public:
		// Solves W = A * Lambda + Bias with projected Gauss-Seidel. The
		// return value reports whether the problem was valid and executed;
		// convergence information is returned separately in Result.
		static bool SolveProjectedGaussSeidel(const LCPProblem& Problem, float* Lambda,
			const LCPSolverSettings& Settings, LCPSolverResult* Result = nullptr);
		static bool IsBackendAvailable(SolverBackend Backend);
		static bool SolveSparseProjectedGaussSeidel(const LCPProblem& Problem,
			const LCPSparseMatrix& Matrix, float* Lambda, const LCPSolverSettings& Settings,
			LCPSolverResult* Result);
	};

	ConstraintSolver* CreateGlobalLCPSolver(const LCPSolverSettings& Settings);
}
