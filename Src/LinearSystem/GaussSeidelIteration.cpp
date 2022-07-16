
#include "GaussSeidelIteration.h"
#include "../Maths/SIMD.h"

// Successive Over - relaxation

bool GaussSeidelIteration_CPU::Solve(const float* A, const float* B, int N, float* X, const int MaxIteration, const float kEps, const float Relaxation)
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
			float delta = (B[i] - beta) / A[i * N + i];
			Norm += (X[i] - delta) * (X[i] - delta);
			X[i] += Relaxation * (delta - X[i]);
		}

		if (Norm < kEps)
		{
			return true;
		}
	}
	return false;
}
