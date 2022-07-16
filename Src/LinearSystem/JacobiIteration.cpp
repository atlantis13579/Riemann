
#include "JacobiIteration.h"
#include "../Maths/SIMD.h"

bool JacobiIteration_CPU::Solve(const float* A, const float* B, int N, float* X, const int MaxIteration, const float kEps)
{
	if (m_buf.size() < N)
	{
		m_buf.resize(N);
	}
	float* X1 = &m_buf[0];

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
			X1[i] = (B[i] - beta) / A[i * N + i];
		}

		float Norm = 0.0f;
		for (int i = 0; i < N; i++)
		{
			Norm += (X[i] - X1[i]) * (X[i] - X1[i]);
		}

		memcpy(X, X1, sizeof(X[0]) * N);

		if (Norm < kEps)
		{
			return true;
		}
	}
	return false;
}
