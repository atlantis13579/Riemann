
#pragma once

// https://en.wikipedia.org/wiki/LU_decomposition
class LUFactorization
{
public:
	static void Solve(const float* A, int N, float* L, float* U)
	{
		memset(U, 0, sizeof(U[0]) * N * N);

		memset(L, 0, sizeof(L[0]) * N * N);
		for (int i = 0; i < N; ++i)
		{
			L[i*N + i] = 1.0f;
		}

		for (int i = 0; i < N; ++i)
		{
			for (int j = i; j < N; ++j)
			{
				float s = 0.0f;
				for (int k = 0; k < i; ++k)
				{
					s += L[i * N + k] * U[k * N + i];
				}
				for (int k = i + 1; k < N; ++k)
				{
					s += L[i * N + k] * U[k * N + i];
				}

				U[i * N+j] = A[i * N + j] - s;
			}

			if (i + 1 < N)
			{
				for (int j = i + 1; j < N; ++j)
				{
					float s = 0.0f;
					for (int k = 0; k < i; ++k)
					{
						s += L[i * N + k] * U[k * N + i];
					}
					for (int k = i+1; k < N; ++k)
					{
						s += L[i * N + k] * U[k * N + i];
					}

					L[j * N + i] = (A[j * N + i] - s) / U[i * N + i];
				}

			}
		}
	}



};