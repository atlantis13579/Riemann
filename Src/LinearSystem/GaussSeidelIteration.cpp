
#include "GaussSeidelIteration.h"
#include "../Maths/SIMD.h"

void GaussSeidelIteration_CPU::Solve(const float* A, const float* B, int N, float* X, const int MaxIteration, const float kEps /*= 0.00001f*/)
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
			X[i] = (B[i] - beta) / A[i * N + i];
		}

		float Norm = 0.0f;
		for (int i = 0; i < N; i++)
		{
			Norm += X[i] * X[i];
		}
		bool converge = Norm < kEps;
		if (converge)
		{
			break;
		}
	}
}
