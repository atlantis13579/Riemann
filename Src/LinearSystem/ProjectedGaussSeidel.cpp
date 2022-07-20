
#include <algorithm>
#include <vector>
#include "ProjectedGaussSeidel.h"

bool ProjectedGaussSeidel_CPU::Solve(const float* A, const float* B, int N, float* X, const int MaxIteration, const float kEps, const float Relaxation)
{
	int Iter = 0;
	while (Iter++ < MaxIteration)
	{
		float Norm = 0.0f;

		for (int i = 0; i < N; ++i)
		{
			float beta = 0.0f;
			for (int j = 0; j < i; ++j)
			{
				beta += A[i * N + j] * X[j];
			}
			for (int j = i + 1; j < N; ++j)
			{
				beta += A[i * N + j] * X[j];
			}
			float X0 = (-B[i] - beta) / A[i * N + i];
			X0 = X[i] + Relaxation * (X0 - X[i]);
			X0 = std::max(0.0f, X0);
			Norm += (X[i] - X0) * (X[i] - X0);
			X[i] = X0;
		}

		if (Norm < kEps)
		{
			return true;
		}
	}
	return false;
}

bool ProjectedGaussSeidel_CPU::Solve(const float* A, const float* B, int N, float* X, const float* X1, const float* X2, const int MaxIteration, const float kEps, const float Relaxation)
{
	int Iter = 0;
	while (Iter++ < MaxIteration)
	{
		float Norm = 0.0f;
		for (int i = 0; i < N; ++i)
		{
			float beta = 0.0f;
			for (int j = 0; j < i; ++j)
			{
				beta += A[i * N + j] * X[j];
			}
			for (int j = i + 1; j < N; ++j)
			{
				beta += A[i * N + j] * X[j];
			}
			float X0 = (-B[i] - beta) / A[i * N + i];
			X0 = X[i] + Relaxation * (X0 - X[i]);
			X0 = std::min(std::max(X1[i], X0), X2[i]);
			Norm += (X[i] - X0) * (X[i] - X0);
			X[i] = X0;
		}

		if (Norm < kEps)
		{
			return true;
		}
	}
	return false;
}
