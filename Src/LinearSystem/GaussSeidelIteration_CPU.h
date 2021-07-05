
#pragma once

// Solve A * X = B , A is square Matrix of size N, B is Vector of size N
// https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method
template<typename T>
class GaussSeidelIteration_CPU
{
public:
	static void Solve(const T* A, const T* B, int N, int MaxIteration, T* X)
	{
		int Iter = 0;
		while (Iter++ < MaxIteration)
		{
			for (int i = 0; i < N; ++i)
			{
				T s = (T)0;
				for (int j = 0; j < i; ++j)
					s += A[i * N + j] * X[j];

				for (int j = i + 1; j < N; ++j)
					s += A[i * N + j] * X[j];

				X[i] = (B[i] - s) / A[i * N + i];
			}

			T Norm = (T)0;
			for (int i = 0; i < N; i++)
				Norm += X[i] * X[i];

			bool converge = Norm < (T)0.00001;
			if (converge)
			{
				break;
			}
		}
	}
};