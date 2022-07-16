
#include <algorithm>
#include "ProjectedGaussSeidel.h"

bool ProjectedGaussSeidel_CPU::Solve(const float* A, const float* B, int N, float* X, const int MaxIteration, const float kEps /*= 0.00001f*/)
{
	int Iter = 0;
	while (Iter++ < MaxIteration)
	{
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
			X[i] = (-B[i] - beta) / A[i * N + i];
		}
		for (int i = 0; i < N; ++i)
		{
			X[i] = std::max(0.0f, X[i]);
		}

		float Norm = 0.0f;
		for (int i = 0; i < N; i++)
		{
			Norm += X[i] * X[i];
		}
		if (Norm < kEps)
		{
			return true;
		}
	}
	return false;
}

bool ProjectedGaussSeidel_CPU::Solve(const float* A, const float* B, const float* X1, const float* X2, int N, float* X, const int MaxIteration, const float kEps /*= 0.00001f*/)
{
	int Iter = 0;
	while (Iter++ < MaxIteration)
	{
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
			X[i] = (-B[i] - beta) / A[i * N + i];
		}
		for (int i = 0; i < N; ++i)
		{
			X[i] = std::min(std::max(X1[i], X[i]), X2[i]);
		}

		float Norm = 0.0f;
		for (int i = 0; i < N; i++)
		{
			Norm += X[i] * X[i];
		}
		if (Norm < kEps)
		{
			return true;
		}
	}
	return false;
}
