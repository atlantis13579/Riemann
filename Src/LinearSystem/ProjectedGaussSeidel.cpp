
#include <algorithm>
#include "ProjectedGaussSeidel.h"

bool ProjectedGaussSeidel_CPU::Solve(const float* A, const float* B, int N, float* X, const int MaxIteration, const float kEps /*= 0.00001f*/)
{
	int Iter = 0;

	float* old = new float[N];

	while (Iter++ < MaxIteration)
	{
		memcpy(old, X, sizeof(float) * N);

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
			float XN = (-B[i] - beta) / A[i * N + i];
			// XN = std::max(0.0f, XN);
			// Norm += (X[i] - XN) * (X[i] - XN);
			X[i] = XN;
		}

		for (int i = 0; i < N; ++i)
		{
			Norm += (X[i] - old[i]) * (X[i] - old[i]);
			X[i] = std::max(0.0f, X[i]);
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
	float* old = new float[N];

	int Iter = 0;
	while (Iter++ < MaxIteration)
	{
		memcpy(old, X, sizeof(float) * N);

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
			float XN = (-B[i] - beta) / A[i * N + i];
			// XN = std::min(std::max(X1[i], XN), X2[i]);
			// Norm += (X[i] - XN) * (X[i] - XN);
			X[i] = XN;
		}

		for (int i = 0; i < N; ++i)
		{
			Norm += (X[i] - old[i]) * (X[i] - old[i]);
			X[i] = std::min(std::max(X1[i], X[i]), X2[i]);
		}

		if (Norm < kEps)
		{
			return true;
		}
	}
	return false;
}
